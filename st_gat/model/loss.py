"""
CombinedLoss for STGAT (RISE edition).

Changes from the T-ITS reference:
  - VAR_FLOOR replaces the commented-out variance clamp: prevents log(0) and
    gradient explosion when variance collapses early in training. The floor is
    consistent with model.py's softplus(var) + VAR_FLOOR, so the floor is
    physically present in the prediction, not just a loss clamp.
  - Variance regularisation weight lowered from 0.01 to 0.001 — avoids
    pushing variance artificially high and conflicting with the NLL objective.
  - Each traffic-light channel has its own loss weight so you can down-weight
    them without touching position/velocity.
  - Added total_nll and total_bce sub-totals for easier tensorboard logging.
  - object_distance's MSE term removed 2026-08-02 — that feature (and
    closest_object_velocity) collapsed every tracked object to a "nearest
    object" heuristic; replaced by objects_set/objects_mask, an input-only
    per-object representation with no output head (see model.py's
    ObjectSetEncoder and docs/theoretical_framework.md §3.1 on why objects
    don't get a negative-evidence residual the way traffic lights do).
  - beta-NLL added 2026-08-02 (Seitzer, Tavakoli, Antic & Martius, "On the
    Pitfalls of Heteroscedastic Uncertainty Estimation with Probabilistic
    Neural Networks," ICLR 2022, arXiv:2203.09168): plain Gaussian NLL's
    gradient w.r.t. the mean is implicitly scaled by 1/variance, so a
    sample the model finds hard can reduce its loss contribution by
    inflating predicted variance instead of by fitting the mean better —
    the model "explains away" error rather than reducing it. Found live: a
    calibration check on this project's first trained model showed every
    Gaussian-headed feature underconfident (predicted std ~0.6-0.95x what
    the actual error spread justified, not ~1.0x), alongside genuinely poor
    point accuracy (1-step position error 30x worse than a trivial
    constant-velocity baseline) — the textbook beta-NLL failure signature,
    not just "needs more training." beta-NLL reweights each sample's NLL
    contribution by its own variance (stop-gradient), raised to BETA_NLL,
    decoupling the mean-fitting gradient from the variance-scaling
    pathology; BETA_NLL=0 recovers plain NLL, BETA_NLL=1 weights samples
    close to how plain MSE would.
  - VAR_REG_WEIGHT (a flat 1/variance penalty against collapse) REMOVED
    2026-08-02. It was error-unaware — the same fixed penalty applied
    whether real error was large or (after the position-representation fix)
    tiny. Found live: once position error dropped from 2.56m to 0.11m,
    this term's constant pull toward larger variance dominated relative to
    the now-much-smaller NLL signal, making position/steering calibration
    MORE underconfident than before the fixes, not less (predicted std
    0.02-0.28x actual error spread, worse than the pre-fix 0.6-0.7x).
    VAR_FLOOR already provides a hard floor against literal collapse
    (log(0)); beta-NLL already addresses the gradient-scaling pathology
    this term was a blunt-instrument defense against. Kept as a git-history
    note, not a live-but-zeroed knob — re-add deliberately, with a value
    tied to actual error scale, if collapse reappears without it.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

VAR_FLOOR = 1e-8      # must match model.py — applied at prediction time. Lowered
                       # from 1e-4 2026-08-02: it was pinning position's predicted
                       # variance (real error ~0.09m needs var ~8e-7, below the old
                       # floor) — see model.py's docstring for the full diagnosis.
BETA_NLL  = 0.5        # Seitzer et al. 2022 — see module docstring

DEFAULT_WEIGHTS = {
    'position':               1.0,
    'velocity':               0.8,
    'steering':               0.5,
    'acceleration':           0.5,
    # Redesigned 2026-08-05 (see config.py's FEATURE_SIZES doc /
    # docs/research_notes/ablation_study_2026.md): replaces the single
    # collapsed traffic_light_state weight — color and confidence now have
    # independent Gaussian heads/loss terms so a pure confidence-degradation
    # fault (tl_confidence) produces distinguishable gradient signal from a
    # color-changing one. traffic_light_detected's BCE term removed entirely
    # — that head (the old "map expects a TL here" feature) was retired,
    # see model.py's STGAT docstring.
    'traffic_light_color':       0.2,   # Gaussian NLL — perception's reported color
    'traffic_light_confidence':  0.2,   # Gaussian NLL — that same reading's confidence
    'traffic_light_discrepancy': 0.2,   # sigmoid output — BCE loss (perception channel)
}


class CombinedLoss(nn.Module):
    """
    Gaussian NLL for continuous outputs (position, velocity, steering,
    acceleration, traffic_light_color, traffic_light_confidence) + BCE for
    the one binary traffic-light output (traffic_light_discrepancy).

    All variances must already have VAR_FLOOR added (as in model.py).
    """

    def __init__(self, weights: dict = None):
        super().__init__()
        self.weights = weights if weights is not None else DEFAULT_WEIGHTS

    def forward(self, pred: dict, target: dict):
        losses = {}
        total  = torch.tensor(0.0, device=next(iter(pred.values())).device)
        nll_sum = torch.tensor(0.0, device=total.device)

        # ── Gaussian NLL outputs ─────────────────────────────────────────────
        for key in ('position', 'velocity', 'steering', 'acceleration',
                    'traffic_light_color', 'traffic_light_confidence'):
            mean_k = f'{key}_mean'
            var_k  = f'{key}_var'
            if mean_k not in pred:
                continue

            var    = pred[var_k]                      # already ≥ VAR_FLOOR
            target_k = target[key]

            # Squeeze scalar features for matching: (B, T, 1) → (B, T)
            if target_k.dim() == 3 and target_k.size(-1) == 1:
                target_k = target_k.squeeze(-1)

            nll = 0.5 * (torch.log(var) + (target_k - pred[mean_k]).pow(2) / var)
            # beta-NLL (Seitzer et al. 2022, see module docstring): weight each
            # sample by its own variance (stop-gradient — the weight itself
            # contributes no gradient, only rescales the NLL term's) so the
            # mean-fitting gradient isn't implicitly suppressed by 1/variance.
            beta_weight = var.detach().pow(BETA_NLL)
            nll = nll * beta_weight
            nll = nll.sum(dim=tuple(range(1, nll.dim()))).mean()

            w = self.weights.get(key, 1.0)
            losses[f'{key}_loss'] = (nll * w).item()
            total    = total    + nll * w
            nll_sum  = nll_sum  + nll * w

        losses['total_nll'] = nll_sum.item()

        # ── Raw accuracy metrics (monitoring only — NOT part of the training
        # objective, no gradient) ────────────────────────────────────────────
        # NLL-based checkpoint selection can prefer an undertrained state
        # where predicted variance hasn't grown to its calibrated size yet:
        # found live, the epoch with the "best" (most negative) total_loss
        # was epoch 1, before the model had learned much of anything, because
        # widening variance to correctly reflect longer-horizon uncertainty
        # increases the log(var) term faster than any accompanying accuracy
        # gain reduces it — a real, calibration-appropriate change that still
        # makes the scalar loss look worse. Track raw position/velocity error
        # directly so Trainer can select/stop on this project's own stated
        # accuracy targets (TODO.md: <1.0m position, <0.5 m/s velocity)
        # instead of an NLL number that can diverge from them.
        with torch.no_grad():
            if 'position_mean' in pred and 'position' in target:
                losses['position_l2_raw'] = torch.norm(
                    pred['position_mean'] - target['position'], dim=-1
                ).mean().item()
            if 'velocity_mean' in pred and 'velocity' in target:
                losses['velocity_l1_raw'] = (
                    pred['velocity_mean'] - target['velocity']
                ).abs().mean().item()

        # ── BCE output: traffic_light_discrepancy (added 2026-08-01) ─────────
        if 'traffic_light_discrepancy' in pred and 'traffic_light_discrepancy' in target:
            tgt = target['traffic_light_discrepancy']
            if tgt.dim() == 3:
                tgt = tgt.squeeze(-1)
            bce = F.binary_cross_entropy(pred['traffic_light_discrepancy'], tgt)
            w   = self.weights.get('traffic_light_discrepancy', 0.2)
            losses['traffic_light_discrepancy_loss'] = (bce * w).item()
            total = total + bce * w

        losses['total_loss'] = total.item()
        return losses, total
