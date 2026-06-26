"""
CombinedLoss for STGAT (RISE edition).

Changes from the T-ITS reference:
  - VAR_FLOOR replaces the commented-out variance clamp: prevents log(0) and
    gradient explosion when variance collapses early in training. The floor is
    consistent with model.py's softplus(var) + VAR_FLOOR, so the floor is
    physically present in the prediction, not just a loss clamp.
  - Variance regularisation weight lowered from 0.01 to 0.001 — avoids
    pushing variance artificially high and conflicting with the NLL objective.
  - object_distance and traffic_light have their own loss weights so you can
    down-weight them without touching position/velocity.
  - Added total_nll and total_bce sub-totals for easier tensorboard logging.
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
    'object_distance':        0.3,   # sigmoid output — MSE loss
    'traffic_light_detected': 0.2,   # sigmoid output — BCE loss
}

VAR_REG_WEIGHT = 0.001   # prevents variance from collapsing to the floor


class CombinedLoss(nn.Module):
    """
    Gaussian NLL for continuous outputs (position, velocity, steering, acceleration)
    + MSE for object_distance + BCE for traffic_light_detected.

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
        for key in ('position', 'velocity', 'steering', 'acceleration'):
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

        # ── MSE output: object_distance ──────────────────────────────────────
        if 'object_distance' in pred and 'object_distance' in target:
            tgt = target['object_distance']
            if tgt.dim() == 3:
                tgt = tgt.squeeze(-1)
            mse = F.mse_loss(pred['object_distance'], tgt)
            w   = self.weights.get('object_distance', 0.3)
            losses['object_distance_loss'] = (mse * w).item()
            total = total + mse * w

        # ── BCE output: traffic_light_detected ───────────────────────────────
        if 'traffic_light_detected' in pred and 'traffic_light_detected' in target:
            tgt = target['traffic_light_detected']
            if tgt.dim() == 3:
                tgt = tgt.squeeze(-1)
            bce = F.binary_cross_entropy(pred['traffic_light_detected'], tgt)
            w   = self.weights.get('traffic_light_detected', 0.2)
            losses['traffic_light_loss'] = (bce * w).item()
            total = total + bce * w

        losses['total_loss'] = total.item()
        return losses, total
