"""
Training entry point for STGAT-RISE.

Usage:
    cd /home/df/Desktop/Kalpit-2026/Risk-Aware-Control
    python3 -m st_gat.train [--epochs 200] [--batch 128] [--lr 4e-4]

Expects pkl files already produced by:
    python3 -m st_gat.pipeline.run_pipeline
"""

import argparse
import sys
import os

import torch
from torch.utils.data import DataLoader

# Reference codebase on path for dataset imports (needed if loading old-format pkl)
_REF_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', 'Graph-Scene-Representation-and-Prediction'))
if _REF_ROOT not in sys.path:
    sys.path.insert(0, _REF_ROOT)

from st_gat.pipeline import config as cfg
from st_gat.model import STGAT, TrajectoryDataset, Trainer


def build_model_config(args) -> dict:
    base = cfg.MODEL_CONFIG.copy()
    base.update({
        'num_epochs':    args.epochs,
        'batch_size':    args.batch,
        'learning_rate': args.lr,
        'dropout_rate':  args.dropout,
        'd_model':       128,
        'd_graph':       128,
        'hidden_size':   128,
        'num_layers':    2,
        'nhead':         4,
        'patience':      20,
        'max_grad_norm': 1.0,
        'weight_decay':  1e-4,
        'device':        torch.device('cuda' if torch.cuda.is_available() else 'cpu'),
    })
    return base


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--epochs',  type=int,   default=200)
    parser.add_argument('--batch',   type=int,   default=128)
    parser.add_argument('--lr',      type=float, default=4e-4)
    parser.add_argument('--dropout', type=float, default=0.15)
    parser.add_argument('--workers', type=int,   default=4)
    parser.add_argument('--out',     type=str,   default=os.path.join(os.path.dirname(__file__), 'checkpoints'))
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
