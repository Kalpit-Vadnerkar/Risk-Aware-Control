"""
Trainer for STGAT (RISE edition).

Improvements over the T-ITS reference Trainer:
  - Gradient clipping (max_norm=1.0)
  - Early stopping with patience
  - AdamW + OneCycleLR
  - Rich tqdm progress: outer epoch bar with live loss/lr, inner batch bar
  - ETA estimation and per-feature loss breakdown printed every N epochs
  - Saves best checkpoint automatically
"""

import os
import time
from typing import Optional

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
        self.num_epochs     = config['num_epochs']
        self.max_grad_norm  = config.get('max_grad_norm', 1.0)
        self.log_every      = log_every

        os.makedirs(checkpoint_dir, exist_ok=True)

        self.criterion = CombinedLoss().to(self.device)

        self.optimizer = torch.optim.AdamW(
            model.parameters(),
            lr           = config['learning_rate'],
            weight_decay = config.get('weight_decay', 1e-4),
        )

        steps_per_epoch = len(train_loader)
        self.scheduler  = torch.optim.lr_scheduler.OneCycleLR(
            self.optimizer,
            max_lr          = config['learning_rate'],
            epochs          = self.num_epochs,
            steps_per_epoch = steps_per_epoch,
            pct_start       = 0.1,
            anneal_strategy = 'cos',
        )

        self.early_stop = EarlyStopping(
            patience  = config.get('patience', 20),
            min_delta = config.get('min_delta', 1e-4),
        )

        self.history = {'train': [], 'val': []}

    # ── Training / validation loops ────────────────────────────────────────

    def _run_epoch(self, loader: DataLoader, train: bool, epoch_bar=None) -> dict:
        self.model.train(train)
        totals: dict = {}
        n_batches = 0
        tag = 'train' if train else 'val  '

        ctx = torch.enable_grad() if train else torch.no_grad()
        with ctx:
            batch_bar = tqdm(
                loader,
                desc      = f'  {tag}',
                leave     = False,
                unit      = 'batch',
                dynamic_ncols = True,
            )
            for batch in batch_bar:
                past, future, graph, _bounds = batch

                past   = {k: v.to(self.device) for k, v in past.items()}
                future = {k: v.to(self.device) for k, v in future.items()}
                graph  = {k: v.to(self.device) for k, v in graph.items()}

                if train:
                    self.optimizer.zero_grad()

                preds = self.model(past, graph)
                losses, loss = self.criterion(preds, future)

                if train:
                    loss.backward()
                    nn.utils.clip_grad_norm_(self.model.parameters(), self.max_grad_norm)
                    self.optimizer.step()
                    self.scheduler.step()

                for k, v in losses.items():
                    totals[k] = totals.get(k, 0.0) + v
                n_batches += 1

                # Live running loss on the batch bar
                batch_bar.set_postfix(loss=f"{totals.get('total_loss', 0) / n_batches:.4f}")

        return {k: v / n_batches for k, v in totals.items()}

    # ── Public API ─────────────────────────────────────────────────────────

    def train(self) -> nn.Module:
        best_val  = float('inf')
        best_path = os.path.join(self.checkpoint_dir, 'best_model.pth')

        n_params = self.model.count_parameters()
        print(f"\n{'='*60}")
        print(f"  STGAT Training")
        print(f"{'='*60}")
        print(f"  Parameters : {n_params:,}")
        print(f"  Device     : {self.device}")
        print(f"  Max epochs : {self.num_epochs}")
        print(f"  Patience   : {self.early_stop.patience}")
        print(f"  Train batches: {len(self.train_loader)}")
        print(f"  Val   batches: {len(self.val_loader)}")
        print(f"  Checkpoint : {self.checkpoint_dir}")
        print(f"{'='*60}\n")

        epoch_bar = tqdm(
            range(1, self.num_epochs + 1),
            desc       = 'Epochs',
            unit       = 'ep',
            dynamic_ncols = True,
        )

        epoch_times = []

        for epoch in epoch_bar:
            t0 = time.time()

            train_losses = self._run_epoch(self.train_loader, train=True,  epoch_bar=epoch_bar)
            val_losses   = self._run_epoch(self.val_loader,   train=False, epoch_bar=epoch_bar)

            elapsed = time.time() - t0
            epoch_times.append(elapsed)

            self.history['train'].append(train_losses)
            self.history['val'].append(val_losses)

            lr        = self.scheduler.get_last_lr()[0]
            tr_loss   = train_losses['total_loss']
            v_loss    = val_losses['total_loss']
            improved  = '★' if v_loss < best_val else ' '
            patience_left = self.early_stop.patience - self.early_stop.counter

            # ETA from mean epoch time
            remaining = self.num_epochs - epoch
            avg_t     = sum(epoch_times[-10:]) / len(epoch_times[-10:])
            eta_s     = avg_t * remaining
            eta_str   = f"{int(eta_s//3600):02d}:{int((eta_s%3600)//60):02d}:{int(eta_s%60):02d}"

            epoch_bar.set_postfix(
                tr    = f"{tr_loss:.4f}",
                val   = f"{v_loss:.4f}",
                lr    = f"{lr:.1e}",
                pat   = patience_left,
                eta   = eta_str,
            )

            # Checkpoint
            if v_loss < best_val:
                best_val = v_loss
                torch.save(self.model.state_dict(), best_path)

            # Periodic per-feature breakdown
            if epoch % self.log_every == 0 or epoch == 1:
                keys = ['position_loss', 'velocity_loss', 'steering_loss',
                        'acceleration_loss', 'object_distance_loss', 'traffic_light_loss']
                breakdown = '  '.join(
                    f"{k.replace('_loss','')[:5]}={val_losses.get(k, 0):.3f}"
                    for k in keys
                )
                tqdm.write(
                    f"  ep {epoch:4d}  "
                    f"train={tr_loss:.4f}  val={v_loss:.4f}  {improved}  "
                    f"| {breakdown}  "
                    f"lr={lr:.1e}  ({elapsed:.1f}s)"
                )

            if self.early_stop.step(v_loss):
                tqdm.write(f"\n[trainer] Early stopping at epoch {epoch} — best val={best_val:.4f}")
                break

        epoch_bar.close()
        tqdm.write(f"\n[trainer] Best val loss: {best_val:.4f} — model at {best_path}")
        self.model.load_state_dict(torch.load(best_path, map_location=self.device, weights_only=True))
        self._save_history()
        return self.model

    def _save_history(self):
        import pandas as pd
        for split in ('train', 'val'):
            pd.DataFrame(self.history[split]).to_csv(
                os.path.join(self.checkpoint_dir, f'{split}_losses.csv'), index=False
            )
