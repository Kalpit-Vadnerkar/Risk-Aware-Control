"""
Training entry point for STGAT-RISE.

Usage (ROS must be sourced BEFORE activating the venv, so rclpy/rosbag2_py
resolve from the system site-packages the venv was built with --system-site-packages against):
    source /opt/ros/humble/setup.bash
    source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
    cd /home/kvadner/Desktop/Dissertation/Risk-Aware-Control
    source .venv/bin/activate
    python3 -m st_gat.train [--epochs 200] [--batch 128] [--lr 4e-4]
        [--warmup-epochs 25] [--phase2-lr 4e-4]

Expects pkl files already produced by:
    python3 -m st_gat.pipeline.run_pipeline

Two-phase training (2026-08-07, see Trainer's own docstring and
docs/research_notes/calibration_training_literature_2026-08-07.md):
--warmup-epochs controls Phase 1 (mean-only warmup, fixed duration, no
early stopping); --epochs/--patience control Phase 2's ceiling (full
Student-t NLL + learned task weighting, early-stopped as before);
--phase2-lr sets Phase 2's own starting LR (defaults to --lr) rather than
inheriting whatever LR decay Phase 1 leaves the optimizer at.
"""

import argparse
import os

import torch
from torch.utils.data import DataLoader

from st_gat.pipeline import config as cfg
from st_gat.model import STGAT, TrajectoryDataset, Trainer


def build_model_config(args) -> dict:
    # Architecture dims (d_model/d_graph/hidden_size/num_layers/nhead) come
    # from cfg.build_inference_model_cfg() -- the same source every
    # inference/analysis script now uses (2026-08-07 consolidation) -- only
    # dropout_rate is still a genuine CLI override here (--dropout), since
    # unlike the others it's actually swept sometimes.
    base = cfg.build_inference_model_cfg()
    base.update({
        'num_epochs':    args.epochs,
        'warmup_epochs': args.warmup_epochs,
        'batch_size':    args.batch,
        'learning_rate': args.lr,
        'phase2_lr':     args.phase2_lr if args.phase2_lr is not None else args.lr,
        'dropout_rate':  args.dropout,
        'patience':      20,
        'max_grad_norm': 1.0,
        'weight_decay':  1e-4,
        'device':        torch.device('cuda' if torch.cuda.is_available() else 'cpu'),
    })
    return base


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--epochs',  type=int,   default=200, help='Phase 2 (finetune) epoch ceiling')
    parser.add_argument('--warmup-epochs', type=int, default=25,
                        help='Phase 1 (mean-only warmup) fixed duration, no early stopping')
    parser.add_argument('--batch',   type=int,   default=128)
    parser.add_argument('--lr',      type=float, default=4e-4, help='Phase 1 starting LR')
    parser.add_argument('--phase2-lr', type=float, default=None,
                        help='Phase 2 starting LR (default: same as --lr)')
    parser.add_argument('--dropout', type=float, default=0.15)
    parser.add_argument('--workers', type=int,   default=4)
    parser.add_argument('--out',     type=str,   default=cfg.CHECKPOINT_DIR,
                        help="Checkpoint dir (default: horizon-tagged, e.g. st_gat/checkpoints/h30_30/)")
    args = parser.parse_args()

    model_cfg = build_model_config(args)

    print(f"[train] device: {model_cfg['device']}")
    print(f"[train] train dir: {cfg.TRAIN_DIR}")
    print(f"[train] cal dir:   {cfg.CAL_DIR}")

    train_ds = TrajectoryDataset(cfg.TRAIN_DIR)
    val_ds   = TrajectoryDataset(cfg.CAL_DIR)

    train_loader = DataLoader(
        train_ds, batch_size=args.batch, shuffle=True,
        num_workers=args.workers, pin_memory=True, drop_last=True,
    )
    val_loader = DataLoader(
        val_ds, batch_size=args.batch, shuffle=False,
        num_workers=args.workers, pin_memory=True,
    )

    model = STGAT(model_cfg)

    trainer = Trainer(
        model          = model,
        train_loader   = train_loader,
        val_loader     = val_loader,
        config         = model_cfg,
        checkpoint_dir = args.out,
    )

    trained_model = trainer.train()

    final_path = model_cfg['model_path']
    os.makedirs(os.path.dirname(final_path), exist_ok=True)
    torch.save(trained_model.state_dict(), final_path)
    print(f"[train] Final model saved to {final_path}")


if __name__ == '__main__':
    main()
