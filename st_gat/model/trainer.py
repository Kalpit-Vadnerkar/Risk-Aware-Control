"""
Trainer for STGAT (RISE edition).

Improvements over the T-ITS reference Trainer:
  - Gradient clipping (max_norm=1.0)
  - Early stopping with patience
  - AdamW + ReduceLROnPlateau
  - Rich tqdm progress: outer epoch bar with live loss/lr, inner batch bar
  - ETA estimation and per-feature loss breakdown printed every N epochs
  - Saves best checkpoint automatically

Scheduler changed 2026-08-02 from OneCycleLR to ReduceLROnPlateau — OneCycleLR
anneals over a FIXED total (`epochs=config['num_epochs']`, the early-stopping
ceiling, not the actual training length), so a run that early-stops well
before that ceiling (e.g. epoch 49 of a 200-epoch schedule, with only a
20-epoch warmup) never reaches the low-LR fine-tuning phase the schedule
promises — LR was still ~98% of peak when training stopped. Found via a
calibration check on the first real trained model: position accuracy missed
this project's own stated target (mean 2.56m vs. TODO.md's <1.0m) and every
Gaussian-headed feature was underconfident (predicted variance wider than the
actual error spread justified) — both consistent with a model that never got
to fine-tune at a low learning rate. ReduceLROnPlateau ties LR reduction to
the same val-loss-plateau signal already driving early stopping, so it
adapts to however long training actually runs instead of assuming a duration
upfront.

Two-phase training added 2026-08-07 (see
docs/research_notes/calibration_training_literature_2026-08-07.md and
loss.py's module docstring point 1): the Gaussian->Student-t retrain gave a
MIXED calibration result, traced to joint mean+scale+dof optimization from a
cold start being fragile (Stirn et al., AISTATS 2023) -- fixing the mean
first, THEN fine-tuning the full distribution, is their prescribed fix.
  - Phase 1 ("warmup"): CombinedLoss.mean_only_forward() -- point-error on
    every head's mean only, fixed duration (config['warmup_epochs'], no
    early stopping -- we WANT this phase to run to completion so scale/dof
    get a stable mean to condition on, not stop the moment point-error
    noisily plateaus).
  - Phase 2 ("finetune"): CombinedLoss.forward() -- full Student-t NLL +
    Kendall et al. 2018 learned task weighting, with a FRESH optimizer/
    scheduler/early-stopping state (config['phase2_lr'], defaulting to the
    same starting LR as Phase 1) so this phase gets real optimization
    budget instead of inheriting whatever LR decay Phase 1's very different
    loss scale left it at. Selection metric is validation total_nll (raw,
    unweighted -- see loss.py), safe to use now that Phase 1 already solved
    the "NLL prefers an undertrained mean" pathology that used to make
    NLL-based selection unusable (see the pre-2026-08-07 version of this
    docstring / v_raw's own comment history).
"""

import os
import time
from typing import Callable, Optional

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm

from .loss import CombinedLoss


class EarlyStopping:
    def __init__(self, patience: int = 20, min_delta: float = 1e-4):
        self.patience  = patience
        self.min_delta = min_delta
        self.best      = float('inf')
        self.counter   = 0

    def step(self, val_loss: float) -> bool:
        if val_loss < self.best - self.min_delta:
            self.best    = val_loss
            self.counter = 0
        else:
            self.counter += 1
        return self.counter >= self.patience


class Trainer:
    def __init__(
        self,
        model:          nn.Module,
        train_loader:   DataLoader,
        val_loader:     DataLoader,
        config:         dict,
        checkpoint_dir: str = 'checkpoints',
        log_every:      int = 10,    # print per-feature breakdown every N epochs
    ):
        self.device         = config['device']
        self.model          = model.to(self.device)
        self.train_loader   = train_loader
        self.val_loader     = val_loader
        self.checkpoint_dir = checkpoint_dir
        self.num_epochs     = config['num_epochs']       # Phase 2 ceiling
        self.warmup_epochs  = config.get('warmup_epochs', 25)   # Phase 1, fixed duration
        self.max_grad_norm  = config.get('max_grad_norm', 1.0)
        self.log_every      = log_every
        self.config         = config

        os.makedirs(checkpoint_dir, exist_ok=True)

        self.criterion = CombinedLoss().to(self.device)

        # Phase 1 optimizer/scheduler/early-stop -- _start_phase() rebuilds
        # all three fresh when Phase 2 begins (see module docstring).
        # Includes self.criterion.parameters() (the learned task-uncertainty
        # weights) as well as the model's -- those otherwise never train,
        # since CombinedLoss is a separate nn.Module from `model`. Harmless
        # in Phase 1 (mean_only_forward doesn't reference them, so they get
        # zero gradient and stay at their neutral init until Phase 2).
        self._build_optimizer_and_schedule(config['learning_rate'])

        self.history = {'warmup': {'train': [], 'val': []}, 'finetune': {'train': [], 'val': []}}

    def _build_optimizer_and_schedule(self, lr: float):
        params = list(self.model.parameters()) + list(self.criterion.parameters())
        self.optimizer = torch.optim.AdamW(
            params, lr=lr, weight_decay=self.config.get('weight_decay', 1e-4),
        )
        # Reduction patience shorter than early-stopping patience so LR
        # actually drops -- giving the model a chance to improve at the
        # lower LR -- before early stopping gives up entirely.
        self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer, mode='min',
            factor=self.config.get('lr_factor', 0.5),
            patience=self.config.get('lr_patience', 7),
            min_lr=self.config.get('min_lr', 1e-6),
        )
        self.early_stop = EarlyStopping(
            patience=self.config.get('patience', 20),
            min_delta=self.config.get('min_delta', 1e-4),
        )

    # ── Training / validation loops ────────────────────────────────────────

    def _run_epoch(self, loader: DataLoader, train: bool, loss_fn: Callable) -> dict:
        self.model.train(train)
        totals: dict = {}
        n_batches = 0
        tag = 'train' if train else 'val  '

        ctx = torch.enable_grad() if train else torch.no_grad()
        with ctx:
            batch_bar = tqdm(loader, desc=f'  {tag}', leave=False, unit='batch', dynamic_ncols=True)
            for batch in batch_bar:
                past, future, graph, _bounds = batch
                past   = {k: v.to(self.device) for k, v in past.items()}
                future = {k: v.to(self.device) for k, v in future.items()}
                graph  = {k: v.to(self.device) for k, v in graph.items()}

                if train:
                    self.optimizer.zero_grad()

                preds = self.model(past, graph)
                losses, loss = loss_fn(preds, future)

                if train:
                    loss.backward()
                    nn.utils.clip_grad_norm_(self.model.parameters(), self.max_grad_norm)
                    self.optimizer.step()

                for k, v in losses.items():
                    totals[k] = totals.get(k, 0.0) + v
                n_batches += 1
                batch_bar.set_postfix(loss=f"{totals.get('total_loss', 0) / n_batches:.4f}")

        return {k: v / n_batches for k, v in totals.items()}

    # ── One phase (warmup or finetune) ──────────────────────────────────────

    def _run_phase(
        self, phase_name: str, loss_fn: Callable, num_epochs: int,
        selection_key: str, use_early_stopping: bool, checkpoint_name: str,
        breakdown_keys: list,
    ) -> float:
        """Runs `num_epochs` of train/val, selecting/checkpointing on
        val_losses[selection_key]. Returns the best value reached (for the
        caller to print / decide phase transitions)."""
        best_val = float('inf')
        best_path = os.path.join(self.checkpoint_dir, checkpoint_name)

        print(f"\n{'='*60}\n  STGAT Training -- Phase: {phase_name}\n{'='*60}")
        print(f"  Epochs (fixed)" if not use_early_stopping else "  Max epochs", f": {num_epochs}")
        if use_early_stopping:
            print(f"  Patience   : {self.early_stop.patience}")
        print(f"  Selection  : {selection_key}")
        print(f"  Checkpoint : {best_path}")
        print(f"{'='*60}\n")

        epoch_bar = tqdm(range(1, num_epochs + 1), desc=f'{phase_name} epochs', unit='ep', dynamic_ncols=True)
        epoch_times = []

        for epoch in epoch_bar:
            t0 = time.time()
            train_losses = self._run_epoch(self.train_loader, train=True,  loss_fn=loss_fn)
            val_losses   = self._run_epoch(self.val_loader,   train=False, loss_fn=loss_fn)
            elapsed = time.time() - t0
            epoch_times.append(elapsed)

            self.history[phase_name]['train'].append(train_losses)
            self.history[phase_name]['val'].append(val_losses)

            v_sel = val_losses.get(selection_key, val_losses.get('total_loss', 0.0))
            self.scheduler.step(v_sel)
            lr = self.optimizer.param_groups[0]['lr']
            improved = '★' if v_sel < best_val else ' '
            patience_left = self.early_stop.patience - self.early_stop.counter if use_early_stopping else '-'

            remaining = num_epochs - epoch
            avg_t = sum(epoch_times[-10:]) / len(epoch_times[-10:])
            eta_s = avg_t * remaining
            eta_str = f"{int(eta_s//3600):02d}:{int((eta_s%3600)//60):02d}:{int(eta_s%60):02d}"

            epoch_bar.set_postfix(
                tr=f"{train_losses.get('total_loss', 0):.4f}",
                val=f"{val_losses.get('total_loss', 0):.4f}",
                sel=f"{v_sel:.4f}", lr=f"{lr:.1e}", pat=patience_left, eta=eta_str,
            )

            if v_sel < best_val:
                best_val = v_sel
                torch.save(self.model.state_dict(), best_path)

            if epoch % self.log_every == 0 or epoch == 1:
                breakdown = '  '.join(f"{k.replace('_loss','').replace('_raw','')[:5]}="
                                       f"{val_losses.get(k, 0):.4f}" for k in breakdown_keys)
                extra = ''
                if phase_name == 'finetune':
                    w = self.criterion.task_weights()
                    extra = '  task_w[' + ' '.join(f"{k[:4]}={v:.2f}" for k, v in w.items()) + ']'
                tqdm.write(
                    f"  [{phase_name}] ep {epoch:4d}  sel={v_sel:.4f}  {improved}  "
                    f"| {breakdown}{extra}  lr={lr:.1e}  ({elapsed:.1f}s)"
                )

            if use_early_stopping and self.early_stop.step(v_sel):
                tqdm.write(f"\n[trainer] {phase_name}: early stopping at epoch {epoch} — best {selection_key}={best_val:.4f}")
                break

        epoch_bar.close()
        tqdm.write(f"\n[trainer] {phase_name} complete — best {selection_key}={best_val:.4f} — model at {best_path}")
        self.model.load_state_dict(torch.load(best_path, map_location=self.device, weights_only=True))
        return best_val

    # ── Public API ─────────────────────────────────────────────────────────

    def train(self) -> nn.Module:
        n_params = self.model.count_parameters()
        print(f"\nSTGAT total parameters: {n_params:,}  |  device: {self.device}")
        print(f"Train batches: {len(self.train_loader)}  |  Val batches: {len(self.val_loader)}")

        # ── Phase 1: mean-only warmup (fixed duration, no early stopping) ──
        warmup_keys = ['position_raw', 'velocity_raw', 'steering_raw', 'acceleration_raw',
                       'traffic_light_color_raw', 'traffic_light_confidence_raw',
                       'traffic_light_discrepancy_loss']
        self._run_phase(
            phase_name='warmup', loss_fn=self.criterion.mean_only_forward,
            num_epochs=self.warmup_epochs, selection_key='total_loss',
            use_early_stopping=False, checkpoint_name='mean_warmup.pth',
            breakdown_keys=warmup_keys,
        )

        # ── Phase transition: fresh optimizer/scheduler/early-stop, so Phase
        # 2 gets its own real LR budget instead of inheriting Phase 1's decay
        # (Phase 1 optimizes a completely different loss scale) ────────────
        phase2_lr = self.config.get('phase2_lr', self.config['learning_rate'])
        tqdm.write(f"\n[trainer] Phase transition: resetting optimizer/scheduler/early-stop, phase2_lr={phase2_lr:.1e}")
        self._build_optimizer_and_schedule(phase2_lr)

        # ── Phase 2: full Student-t NLL + learned task weighting ───────────
        finetune_keys = ['position_loss', 'velocity_loss', 'steering_loss',
                          'acceleration_loss', 'traffic_light_color_loss',
                          'traffic_light_confidence_loss', 'traffic_light_discrepancy_loss']
        self._run_phase(
            phase_name='finetune', loss_fn=self.criterion,
            num_epochs=self.num_epochs, selection_key='total_nll',
            use_early_stopping=True, checkpoint_name='best_model.pth',
            breakdown_keys=finetune_keys,
        )

        tqdm.write(f"\n[trainer] Final learned task weights: {self.criterion.task_weights()}")
        self._save_history()
        return self.model

    def _save_history(self):
        import pandas as pd
        for phase in ('warmup', 'finetune'):
            for split in ('train', 'val'):
                rows = self.history[phase][split]
                if not rows:
                    continue
                pd.DataFrame(rows).to_csv(
                    os.path.join(self.checkpoint_dir, f'{phase}_{split}_losses.csv'), index=False
                )
