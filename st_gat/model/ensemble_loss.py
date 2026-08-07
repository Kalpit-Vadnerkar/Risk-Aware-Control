"""
GaussianMemberLoss — loss for a single STGATEnsemble member (RISE edition).

See docs/research_notes/model_redesign_literature_2026-08-07.md for the
full design rationale. Each ensemble member uses Gaussian heads
(model.py's STGAT(distribution='gaussian')) rather than the single-network
Student-t heads (loss.py's CombinedLoss) -- deliberately simpler than that
path in one specific way: FIXED task weights (DEFAULT_WEIGHTS below), not
the Kendall et al. learned per-task weighting that ran away in a live run
2026-08-07 (see loss.py's module docstring for the full incident). Ensemble
diversity is supposed to come from independent initialization and (later)
data order, not from a per-member adaptive weighting scheme -- removing
that mechanism here removes the exact failure mode found, rather than
requiring it to be perfectly re-tuned before it can be trusted again.

Deliberately kept as an independent module, not built on top of loss.py's
CombinedLoss, so ensemble training isn't coupled to whatever the
single-network Student-t path needs to do next -- some duplication
(mean_only_forward is identical in spirit) is an accepted cost of that
independence, not an oversight.

Kept from the Student-t path, because both are validated fixes to real
problems that are NOT specific to the distribution family:
  - beta-NLL reweighting (Seitzer et al. 2022, ICLR) -- stop-gradient
    weighting by each sample's own predicted variance, so the mean-fitting
    gradient isn't implicitly suppressed by a wide predictive spread.
  - VAR_REG_WEIGHT (added 2026-08-07, loss.py) -- an asymmetric,
    per-timestep, per-feature penalty that only fires when predicted
    variance falls below that batch's own empirical residual magnitude at
    that timestep, computed fresh every batch. This is the regularizer
    that gave a modest but real improvement in the single-network sweep
    (docs/research_notes/calibration_training_literature_2026-08-07.md);
    its strength has NOT yet been separately swept for the ensemble/
    Gaussian setting specifically -- the value below is carried over from
    that sweep's best-performing candidate, not independently verified here.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

SCALE_FLOOR = 1e-4    # must match model.py
BETA_NLL = 0.5
VAR_REG_WEIGHT = 1.0   # carried over from the single-network sweep's best
                       # candidate (docs/research_notes/calibration_training_
                       # literature_2026-08-07.md) -- NOT yet independently
                       # verified for the ensemble/Gaussian setting.

_LOG2PI = 1.8378770664093453   # math.log(2 * math.pi)

DEFAULT_WEIGHTS = {
    'position':                  1.0,
    'velocity':                  0.8,
    'steering':                  0.5,
    'acceleration':              0.5,
    'traffic_light_color':       0.2,
    'traffic_light_confidence':  0.2,
    'traffic_light_discrepancy': 0.2,
}


def _gaussian_nll(target: torch.Tensor, mean: torch.Tensor, var: torch.Tensor) -> torch.Tensor:
    """Elementwise Gaussian NLL (caller sums over dims/timesteps)."""
    return 0.5 * (_LOG2PI + torch.log(var) + (target - mean).pow(2) / var)


class GaussianMemberLoss(nn.Module):
    """Gaussian NLL + beta-NLL + VAR_REG_WEIGHT + fixed task weights, for
    one STGATEnsemble member. Two entry points, same two-phase convention
    as loss.py's CombinedLoss: mean_only_forward() for Phase 1 (mean-only
    warmup), forward() for Phase 2 (full Gaussian NLL)."""

    def __init__(self, weights: dict = None):
        super().__init__()
        self.weights = weights if weights is not None else DEFAULT_WEIGHTS

    def mean_only_forward(self, pred: dict, target: dict):
        """Identical in spirit to CombinedLoss.mean_only_forward -- plain
        point-error on each continuous head's mean, unweighted sum, no
        variance/scale gradient at all. Duplicated rather than imported,
        deliberately (see module docstring)."""
        losses = {}
        total = torch.tensor(0.0, device=next(iter(pred.values())).device)

        for key in ('position', 'velocity', 'steering', 'acceleration',
                    'traffic_light_color', 'traffic_light_confidence'):
            mean_k = f'{key}_mean'
            if mean_k not in pred:
                continue
            target_k = target[key]
            if target_k.dim() == 3 and target_k.size(-1) == 1:
                target_k = target_k.squeeze(-1)
            err = pred[mean_k] - target_k
            point_err = torch.norm(err, dim=-1).mean() if err.dim() == 3 else err.abs().mean()
            losses[f'{key}_raw'] = point_err.item()
            total = total + point_err

        if 'traffic_light_discrepancy' in pred and 'traffic_light_discrepancy' in target:
            tgt = target['traffic_light_discrepancy']
            if tgt.dim() == 3:
                tgt = tgt.squeeze(-1)
            bce = F.binary_cross_entropy(pred['traffic_light_discrepancy'], tgt)
            losses['traffic_light_discrepancy_loss'] = bce.item()
            total = total + bce

        losses['total_loss'] = total.item()
        return losses, total

    def forward(self, pred: dict, target: dict):
        losses = {}
        total   = torch.tensor(0.0, device=next(iter(pred.values())).device)
        nll_sum = torch.tensor(0.0, device=total.device)

        for key in ('position', 'velocity', 'steering', 'acceleration',
                    'traffic_light_color', 'traffic_light_confidence'):
            mean_k = f'{key}_mean'
            var_k  = f'{key}_var'
            if mean_k not in pred:
                continue

            mean = pred[mean_k]
            var  = pred[var_k]                    # already >= SCALE_FLOOR (model.py's floor)
            target_k = target[key]
            if target_k.dim() == 3 and target_k.size(-1) == 1:
                target_k = target_k.squeeze(-1)

            # VAR_REG_WEIGHT (see module docstring) -- per-timestep empirical
            # residual magnitude, reduced over the BATCH only, detached.
            with torch.no_grad():
                empirical_var = (target_k - mean).pow(2).mean(dim=0).clamp(min=SCALE_FLOOR)
            underdispersion = F.relu(empirical_var.unsqueeze(0) - var)
            var_reg = underdispersion.pow(2).mean()
            losses[f'{key}_var_reg'] = var_reg.item()

            nll = _gaussian_nll(target_k, mean, var)
            # beta-NLL (Seitzer et al. 2022) -- stop-gradient weight by each
            # sample's own predicted variance, decoupling the mean-fitting
            # gradient from the variance-scaling pathology.
            beta_weight = var.detach().pow(BETA_NLL)
            nll = nll * beta_weight
            nll = nll.sum(dim=tuple(range(1, nll.dim()))).mean()

            w = self.weights.get(key, 1.0)
            losses[f'{key}_loss'] = (nll * w).item()
            total   = total   + nll * w + VAR_REG_WEIGHT * var_reg
            nll_sum = nll_sum + nll

        losses['total_nll'] = nll_sum.item()

        with torch.no_grad():
            if 'position_mean' in pred and 'position' in target:
                losses['position_l2_raw'] = torch.norm(
                    pred['position_mean'] - target['position'], dim=-1
                ).mean().item()
            if 'velocity_mean' in pred and 'velocity' in target:
                losses['velocity_l1_raw'] = (
                    pred['velocity_mean'] - target['velocity']
                ).abs().mean().item()

        if 'traffic_light_discrepancy' in pred and 'traffic_light_discrepancy' in target:
            tgt = target['traffic_light_discrepancy']
            if tgt.dim() == 3:
                tgt = tgt.squeeze(-1)
            bce = F.binary_cross_entropy(pred['traffic_light_discrepancy'], tgt)
            w = self.weights.get('traffic_light_discrepancy', 0.2)
            losses['traffic_light_discrepancy_loss'] = (bce * w).item()
            total = total + bce * w

        losses['total_loss'] = total.item()
        return losses, total
