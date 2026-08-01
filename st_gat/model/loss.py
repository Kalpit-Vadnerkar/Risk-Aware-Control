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
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

VAR_FLOOR = 1e-4      # must match model.py — applied at prediction time

DEFAULT_WEIGHTS = {
    'position':               1.0,
    'velocity':               0.8,
    'steering':               0.5,
    'acceleration':           0.5,
    'traffic_light_detected': 0.2,   # sigmoid output — BCE loss (map channel)
    # Added 2026-08-01 (docs/stgat_pipeline_plan.md §1.12) — the
    # perception-report channel this dissertation's negative-evidence
    # mechanism actually depends on; previously had no loss term at all
    # since the model had no head for it.
    'traffic_light_state':       0.2,   # Gaussian NLL — perception color x confidence
    'traffic_light_discrepancy': 0.2,   # sigmoid output — BCE loss (perception channel)
}

VAR_REG_WEIGHT = 0.001   # prevents variance from collapsing to the floor


class CombinedLoss(nn.Module):
    """
    Gaussian NLL for continuous outputs (position, velocity, steering,
    acceleration, traffic_light_state) + BCE for the two binary
    traffic-light outputs (traffic_light_detected, traffic_light_discrepancy).

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
                    'traffic_light_state'):
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
            nll = nll.sum(dim=tuple(range(1, nll.dim()))).mean()

            # Mild penalty if variance collapses toward the floor
            var_reg = VAR_REG_WEIGHT * (1.0 / var).mean()
            nll = nll + var_reg

            w = self.weights.get(key, 1.0)
            losses[f'{key}_loss'] = (nll * w).item()
            total    = total    + nll * w
            nll_sum  = nll_sum  + nll * w

        losses['total_nll'] = nll_sum.item()

        # ── BCE output: traffic_light_detected ───────────────────────────────
        if 'traffic_light_detected' in pred and 'traffic_light_detected' in target:
            tgt = target['traffic_light_detected']
            if tgt.dim() == 3:
                tgt = tgt.squeeze(-1)
            bce = F.binary_cross_entropy(pred['traffic_light_detected'], tgt)
            w   = self.weights.get('traffic_light_detected', 0.2)
            losses['traffic_light_loss'] = (bce * w).item()
            total = total + bce * w

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
