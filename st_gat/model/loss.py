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
  - Gaussian NLL -> Student-t NLL (2026-08-06, see model.py's docstring for
    the full reasoning): calibration diagnostics decisively rejected
    normality for every Gaussian-headed feature (leptokurtic residuals),
    which a Gaussian NLL can never fix regardless of training quality — it
    can only ever match the first two moments, not the shape. _student_t_nll
    below replaces the Gaussian NLL term; beta-NLL's motivation (don't let
    the mean-fitting gradient get implicitly suppressed by a large spread
    parameter) carries over using the predicted scale as the difficulty
    proxy, same as before.
  - Two-phase training + learned task weighting (2026-08-07, see
    docs/research_notes/calibration_training_literature_2026-08-07.md):
    the first Student-t retrain gave a MIXED result (better for
    position/steering, worse for acceleration/both TL heads, horizon
    widening still broken for 5/6 heads) — traced to two literature-
    documented causes, not just "needs more training":
      1. Joint mean+scale+dof optimization from a cold start is fragile
         (Stirn et al., "Faithful Heteroscedastic Regression with Neural
         Networks," AISTATS 2023) — the NLL objective can yield BOTH a
         worse mean AND uncalibrated variance vs. an equivalent mean-only
         model, because NLL implicitly down-weights gradient for samples
         already assigned high predicted spread. Fix: mean_only_forward()
         below is Phase 1 (fit the mean via plain point-error, no
         scale/dof gradient at all); forward() below is Phase 2 (full
         Student-t NLL, all parameters), run only after Phase 1 — see
         trainer.py for the two-phase loop this implements.
      2. DEFAULT_WEIGHTS were fixed, hand-picked constants, never adjusted
         for how each feature's training was actually progressing — a
         real instance of the multi-task "loss imbalance" problem
         (notably, the three features that got worse in the mixed retrain
         were exactly the three with the lowest fixed weights). Replaced
         with Kendall, Gal & Cipolla's learned per-task homoscedastic
         uncertainty weighting ("Multi-Task Learning Using Uncertainty to
         Weigh Losses," CVPR 2018): each task gets one learned scalar
         log_task_var, loss_task_weighted = exp(-log_task_var)*loss_task +
         log_task_var — the precision term scales the task's gradient
         contribution, the log term is what stops the trivial "inflate
         log_task_var to zero out this task" collapse. Only active in
         Phase 2 (mean_only_forward doesn't use it) since the mechanism is
         specifically about balancing heterogeneous NLL/BCE tasks, not
         point-errors. Trainer.py MUST include this module's own
         parameters (self.criterion.parameters(), not just the model's) in
         the optimizer for these to actually learn.
"""

import math

import torch
import torch.nn as nn
import torch.nn.functional as F

SCALE_FLOOR = 1e-4    # must match model.py — sqrt of the old VAR_FLOOR=1e-8,
                       # now in scale (sigma) units since heads predict scale
                       # directly rather than variance.
DOF_FLOOR = 2.1        # must match model.py — see its docstring.
BETA_NLL  = 0.5        # Seitzer et al. 2022 — see module docstring


def _student_t_nll(target: torch.Tensor, mean: torch.Tensor, scale: torch.Tensor,
                    dof: torch.Tensor) -> torch.Tensor:
    """Negative log-likelihood of `target` under Student-t(mean, scale, dof),
    elementwise (caller sums over dims/timesteps). dof must already be >
    DOF_FLOOR (as produced by model.py's heads) — finite variance, no
    log(dof-2) singularity risk here since this function doesn't need the
    variance itself, only the density.

    log p(x) = lgamma((dof+1)/2) - lgamma(dof/2) - 0.5*log(dof*pi) - log(scale)
               - ((dof+1)/2) * log1p( ((x-mean)/scale)^2 / dof )
    """
    z2 = ((target - mean) / scale) ** 2
    log_prob = (
        torch.lgamma((dof + 1) / 2) - torch.lgamma(dof / 2)
        - 0.5 * torch.log(dof * math.pi) - torch.log(scale)
        - ((dof + 1) / 2) * torch.log1p(z2 / dof)
    )
    return -log_prob

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


_TASK_KEYS = ('position', 'velocity', 'steering', 'acceleration',
              'traffic_light_color', 'traffic_light_confidence',
              'traffic_light_discrepancy')


class CombinedLoss(nn.Module):
    """
    Student-t NLL for continuous outputs (position, velocity, steering,
    acceleration, traffic_light_color, traffic_light_confidence) + BCE for
    the one binary traffic-light output (traffic_light_discrepancy).

    Scale/dof must already have SCALE_FLOOR/DOF_FLOOR applied (as in model.py).

    Two entry points (2026-08-07, see module docstring): mean_only_forward()
    for Phase 1 (mean-only warmup), forward() for Phase 2 (full NLL +
    learned Kendall-et-al. task weighting) -- trainer.py's two-phase loop
    calls each in turn, not this module deciding which phase it's in.
    """

    def __init__(self, weights: dict = None, task_keys=_TASK_KEYS):
        super().__init__()
        self.weights = weights if weights is not None else DEFAULT_WEIGHTS
        # Kendall et al. 2018 learned per-task uncertainty (see module
        # docstring point 2) -- starts at log_var=0 (weight exp(0)=1.0,
        # neutral) for every task, replacing DEFAULT_WEIGHTS' fixed
        # constants once Phase 2 is active. `weights`/DEFAULT_WEIGHTS above
        # are kept only as Phase 1's implicit equal-ish starting point (Phase
        # 1 doesn't use either -- see mean_only_forward) and as a documented
        # historical reference for what was hand-picked before this fix.
        self.log_task_var = nn.ParameterDict({
            k: nn.Parameter(torch.zeros(1)) for k in task_keys
        })

    def task_weights(self) -> dict:
        """Current learned precision (exp(-log_task_var)) per task -- for
        logging/diagnostics, so the actual learned weighting is visible,
        not just assumed to be doing something reasonable."""
        return {k: float(torch.exp(-v).item()) for k, v in self.log_task_var.items()}

    def mean_only_forward(self, pred: dict, target: dict):
        """Phase 1 ('mean warmup', see module docstring point 1): plain
        point-error on each continuous head's MEAN only -- no scale/dof
        gradient at all, no Student-t NLL, no learned task weighting
        (that mechanism is specifically for balancing heterogeneous NLL/BCE
        losses in Phase 2, not point-errors). Unweighted sum across
        features -- config.py already scales every feature into
        comparable [0,1]-ish ranges, so this is a reasonable simplification
        for a warmup phase, not the final calibration-sensitive objective.
        traffic_light_discrepancy's BCE is included unchanged (that head
        has no mean/variance split to warm up separately)."""
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
            # L2 across feature dims (matches position_l2_raw's convention)
            # for multi-dim features, plain L1 for scalars.
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
        total  = torch.tensor(0.0, device=next(iter(pred.values())).device)
        nll_sum = torch.tensor(0.0, device=total.device)

        # ── Student-t NLL outputs (2026-08-06, see module docstring) ─────────
        for key in ('position', 'velocity', 'steering', 'acceleration',
                    'traffic_light_color', 'traffic_light_confidence'):
            mean_k  = f'{key}_mean'
            scale_k = f'{key}_scale'
            dof_k   = f'{key}_dof'
            if mean_k not in pred:
                continue

            mean  = pred[mean_k]
            scale = pred[scale_k]                     # already ≥ SCALE_FLOOR
            dof   = pred[dof_k]                        # already > DOF_FLOOR, shape (B, T) always
            target_k = target[key]

            # Squeeze scalar features for matching: (B, T, 1) → (B, T)
            if target_k.dim() == 3 and target_k.size(-1) == 1:
                target_k = target_k.squeeze(-1)

            # dof is shared across a multi-dim feature's own dims (e.g.
            # position's x/y) — broadcast (B,T) -> (B,T,1) only when mean
            # actually has an extra dim to broadcast against.
            dof_b = dof.unsqueeze(-1) if mean.dim() == 3 else dof

            nll = _student_t_nll(target_k, mean, scale, dof_b)
            # beta-NLL (Seitzer et al. 2022, see module docstring): weight each
            # sample by its own predicted scale (stop-gradient — the weight
            # itself contributes no gradient, only rescales the NLL term) so
            # the mean-fitting gradient isn't implicitly suppressed by a wide
            # predictive spread. Uses scale^(2*beta) as the (variance-like)
            # difficulty proxy, deliberately not scale^2 * dof/(dof-2) (the
            # true Student-t variance) — simpler and avoids a dof-2 blowup
            # risk near DOF_FLOOR in a term that's only a training heuristic,
            # not part of the likelihood itself.
            beta_weight = scale.detach().pow(2 * BETA_NLL)
            nll = nll * beta_weight
            nll = nll.sum(dim=tuple(range(1, nll.dim()))).mean()

            # Kendall et al. 2018 learned task weighting (2026-08-07, see
            # module docstring point 2) replaces the old fixed
            # self.weights.get(key, 1.0) constant. precision = exp(-log_var)
            # scales this task's gradient contribution; the +log_var term is
            # what stops the trivial "inflate log_var to zero out this
            # task" collapse -- both ARE differentiated through (unlike
            # beta_weight above, which is a detached heuristic).
            log_var   = self.log_task_var[key]
            precision = torch.exp(-log_var)
            weighted  = precision * nll + log_var.squeeze()
            losses[f'{key}_loss'] = weighted.item()
            total    = total    + weighted
            # nll_sum/total_nll stays the RAW (unweighted) sum -- this is
            # what Phase 2 checkpoint selection tracks (see trainer.py),
            # deliberately NOT the learned-weighted objective, so the
            # selection metric doesn't move every time the learned weights
            # shift scale.
            nll_sum  = nll_sum  + nll

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
            log_var   = self.log_task_var['traffic_light_discrepancy']
            precision = torch.exp(-log_var)
            weighted  = precision * bce + log_var.squeeze()
            losses['traffic_light_discrepancy_loss'] = weighted.item()
            losses['traffic_light_discrepancy_bce_raw'] = bce.item()
            total = total + weighted

        losses['total_loss'] = total.item()
        return losses, total
