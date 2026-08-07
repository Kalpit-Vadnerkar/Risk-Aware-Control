"""
STGATEnsemble — deep ensemble of the ST-GAT backbone (RISE edition, added
2026-08-07).

See docs/research_notes/model_redesign_literature_2026-08-07.md for the
full literature review and design rationale this implements. Short version:
three same-day attempts to fix a single Student-t network's calibration
(two-phase training, learned task weighting, a variance-collapse
regularizer) each found and partially addressed a real problem, but none
produced a cleanly-calibrated model -- and the specific failure mode
(joint mean+scale+dof optimization from a cold start driving validation
NLL to degrade monotonically) is a DOCUMENTED, BENCHMARKED property of
single-network heteroscedastic regression (Wong-Toi et al., UAI 2023),
not something this project's particular implementation got wrong. Deep
ensembles are the literature's own empirically-strongest answer to exactly
this project's actual failure scenario -- calibration holding up under
dataset shift (Ovadia et al., NeurIPS 2019: ensembles/methods that
"marginalize over models" were the standout performers in a large benchmark
spanning image, text, and tabular modalities under shift) -- for a
mechanistic reason (Fort, Hu & Lakshminarayanan, 2019: independently-
initialized ensemble members explore genuinely different loss-landscape
modes, so there's no single network's likelihood surface for the collapse
this project hit to exploit).

This is NOT a from-scratch architecture in the sense of discarding the
validated ST-GAT backbone (graph encoder + temporal attention + LSTM +
per-horizon-step-conditioned decoder) -- none of that was ever shown to be
the problem. It IS from-scratch in the sense that matters: how uncertainty
is PRODUCED changes from "one network's self-reported likelihood
parameters" to "disagreement across independently-trained networks," with
a real, literature-standard aleatoric/epistemic decomposition
(Lakshminarayanan et al. 2017's law-of-total-variance combination; Kendall
& Gal 2017's aleatoric/epistemic framing) that the single-network design
had no equivalent of at all -- every uncertainty number produced by this
project before today was purely aleatoric.

Each member: STGAT(distribution='gaussian') (model.py) -- Gaussian, not
Student-t, heads (see model.py's docstring for why, and the design doc's
falsifiable hypothesis that ensemble disagreement may absorb some of what
the single-network redesign needed heavy tails to represent). Each member
trained independently via ensemble_loss.py's GaussianMemberLoss (two-phase,
fixed task weights, NOT the Kendall et al. learned weighting that ran away
in the single-network attempt).
"""

import os

import torch
import torch.nn as nn

from .model import STGAT

_CONTINUOUS_FEATURES = {
    'position': 2, 'velocity': 2, 'steering': 1, 'acceleration': 1,
    'traffic_light_color': 1, 'traffic_light_confidence': 1,
}


class STGATEnsemble(nn.Module):
    """Holds M independently-initialized STGAT(distribution='gaussian')
    members. NOT itself trained end-to-end -- see train_ensemble.py (or
    Trainer, called once per member) for training; this class is for
    combined inference only, run after each member is trained separately.
    """

    def __init__(self, model_cfg: dict, n_members: int = 5):
        super().__init__()
        assert model_cfg.get('distribution', 'gaussian') == 'gaussian', (
            "STGATEnsemble members must use distribution='gaussian' -- see "
            "this module's docstring for why (law-of-total-variance combination "
            "is exact for a Gaussian mixture, not for a mixture of Student-t's)."
        )
        member_cfg = dict(model_cfg, distribution='gaussian')
        self.members = nn.ModuleList([STGAT(member_cfg) for _ in range(n_members)])
        self.n_members = n_members

    @classmethod
    def from_checkpoints(cls, model_cfg: dict, checkpoint_paths: list, device=None):
        """Build an ensemble and load each member from its own trained
        checkpoint (one Trainer run per member -- see train_ensemble.py).
        checkpoint_paths order determines member order; doesn't matter for
        the symmetric combination below."""
        ens = cls(model_cfg, n_members=len(checkpoint_paths))
        for member, path in zip(ens.members, checkpoint_paths):
            member.load_state_dict(torch.load(path, map_location=device, weights_only=True))
        if device is not None:
            ens.to(device)
        return ens

    def forward(self, x: dict, graph: dict) -> dict:
        """Runs every member, combines via the law of total variance
        (Lakshminarayanan et al. 2017): for each continuous feature,
            mean          = average of member means
            aleatoric_var = average of member predicted variances
            epistemic_var = variance OF the member means (disagreement)
            total_var     = aleatoric_var + epistemic_var
        All four returned as separate keys -- the aleatoric/epistemic split
        is itself the new signal this redesign is FOR (see module
        docstring): a genuinely novel situation (a real fault) should show
        epistemic_var responding; ordinary sensor/process noise should
        mostly show up in aleatoric_var instead. Collapsing them back into
        one number would throw away exactly the distinction that motivated
        building this at all.

        traffic_light_discrepancy (Bernoulli, not part of _CONTINUOUS_FEATURES)
        combines the same way conceptually: mean_prob = average of member
        probabilities, epistemic_disagreement = variance of member
        probabilities -- the aleatoric component isn't separately meaningful
        for a single Bernoulli parameter the way it is for a Gaussian, so
        only these two are reported for that head, not a matching
        aleatoric/total split.
        """
        member_outputs = [m(x, graph) for m in self.members]

        out = {}
        for key, dims in _CONTINUOUS_FEATURES.items():
            means = torch.stack([mo[f'{key}_mean'] for mo in member_outputs], dim=0)
            vars_ = torch.stack([mo[f'{key}_var']  for mo in member_outputs], dim=0)

            mean = means.mean(dim=0)
            aleatoric_var = vars_.mean(dim=0)
            epistemic_var = ((means - mean) ** 2).mean(dim=0)
            total_var = aleatoric_var + epistemic_var

            out[f'{key}_mean']          = mean
            out[f'{key}_aleatoric_var'] = aleatoric_var
            out[f'{key}_epistemic_var'] = epistemic_var
            out[f'{key}_var']           = total_var   # kept under the old name too,
            # so existing single-network-shaped consumers (residuals.py's
            # z-score-style checks, plotting) can read a drop-in "the
            # variance" without needing to know the ensemble's internal
            # decomposition -- they just lose the aleatoric/epistemic
            # detail, not correctness, if used unchanged this way.

        probs = torch.stack([mo['traffic_light_discrepancy'] for mo in member_outputs], dim=0)
        out['traffic_light_discrepancy']              = probs.mean(dim=0)
        out['traffic_light_discrepancy_epistemic_var'] = probs.var(dim=0, unbiased=False)

        return out

    def count_parameters(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
