"""
Training entry point for STGATEnsemble members (RISE edition, 2026-08-07).

Trains --n-members independently-initialized STGAT(distribution='gaussian')
members, one Trainer run each (sequential, on this single GPU -- see below
for why not parallel yet), each in its own checkpoint subdir
(<out>/member_0/, member_1/, ...). Each member gets its own random seed
(--seed-base + member index), set BEFORE model construction and BEFORE the
DataLoaders are built, so members differ in both weight initialization and
minibatch/shuffle order -- Lakshminarayanan et al. 2017's own finding was
that random initialization alone (no bootstrap resampling of the training
set) is enough for real ensemble diversity, so that's what this does; see
docs/research_notes/model_redesign_literature_2026-08-07.md.

This script does NOT run STGATEnsemble itself -- that class (ensemble.py)
is inference-only, combining already-trained member checkpoints via the
law of total variance. Run this first, then STGATEnsemble.from_checkpoints().

Usage (ROS/Autoware + repo venv sourced, same as train.py):
  python3 -m st_gat.train_ensemble [--n-members 5] [--epochs 200]
      [--warmup-epochs 25] [--batch 128] [--lr 4e-4]
      [--start-member 2]   # resume: skip members already trained

Compute cost, stated plainly: N members trained sequentially here means
roughly N x a single training run's cost (this project's actual single-
network runs have taken ~2-4 hours each) -- for the default 5 members,
potentially 10-20+ hours total. Members are architecturally independent
(no shared state during training), so running them concurrently as
separate processes IS possible if GPU memory allows (this project's runs
have stayed under ~1.5GB of 49GB available) and would cut wall-clock
substantially -- not implemented as automatic parallelism here, since
sequential is simpler to reason about and resume (--start-member) if
interrupted; a concurrent launcher can be layered on top of this script
(run N copies with different --start-member/--n-members ranges) without
changing anything in here.
"""

import argparse
import os

import torch
from torch.utils.data import DataLoader

from st_gat.pipeline import config as cfg
from st_gat.model import STGAT, TrajectoryDataset, Trainer, GaussianMemberLoss


def build_member_model_cfg(args) -> dict:
    base = cfg.build_inference_model_cfg()
    base.update({
        'distribution':   'gaussian',
        'num_epochs':     args.epochs,
        'warmup_epochs':  args.warmup_epochs,
        'batch_size':     args.batch,
        'learning_rate':  args.lr,
        'phase2_lr':      args.phase2_lr if args.phase2_lr is not None else args.lr,
        'dropout_rate':   args.dropout,
        'patience':       20,
        'max_grad_norm':  1.0,
        'weight_decay':   1e-4,
        'device':         torch.device('cuda' if torch.cuda.is_available() else 'cpu'),
    })
    return base


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--n-members', type=int, default=5,
                        help='ensemble size (Lakshminarayanan et al. and the AV-safety-specific '
                             'Grewal et al. study both report diminishing returns past ~5)')
    parser.add_argument('--epochs',        type=int,   default=200, help='Phase 2 (finetune) epoch ceiling, per member')
    parser.add_argument('--warmup-epochs', type=int,   default=25,  help='Phase 1 duration, per member')
    parser.add_argument('--batch',         type=int,   default=128)
    parser.add_argument('--lr',            type=float, default=4e-4)
    parser.add_argument('--phase2-lr',     type=float, default=None)
    parser.add_argument('--dropout',       type=float, default=0.15)
    parser.add_argument('--workers',       type=int,   default=4)
    parser.add_argument('--out',           type=str,   default=os.path.join(cfg.CHECKPOINT_DIR, 'ensemble'))
    parser.add_argument('--start-member',  type=int,   default=0,
                        help='resume from this member index (skip earlier ones, e.g. after an interruption)')
    parser.add_argument('--seed-base',     type=int,   default=0)
    args = parser.parse_args()

    model_cfg = build_member_model_cfg(args)

    print(f"[train_ensemble] device: {model_cfg['device']}")
    print(f"[train_ensemble] {args.n_members} members, starting at member {args.start_member}")
    print(f"[train_ensemble] checkpoints -> {args.out}/member_*/")

    train_ds = TrajectoryDataset(cfg.TRAIN_DIR)
    val_ds   = TrajectoryDataset(cfg.CAL_DIR)

    for i in range(args.start_member, args.n_members):
        print(f"\n{'#'*70}\n# Member {i + 1}/{args.n_members}\n{'#'*70}")
        torch.manual_seed(args.seed_base + i)   # BEFORE model + DataLoader construction --
        # gives this member its own weight init AND its own shuffle order.

        train_loader = DataLoader(
            train_ds, batch_size=args.batch, shuffle=True,
            num_workers=args.workers, pin_memory=True, drop_last=True,
        )
        val_loader = DataLoader(
            val_ds, batch_size=args.batch, shuffle=False,
            num_workers=args.workers, pin_memory=True,
        )

        model = STGAT(model_cfg)
        member_ckpt_dir = os.path.join(args.out, f'member_{i}')
        trainer = Trainer(
            model=model, train_loader=train_loader, val_loader=val_loader,
            config=model_cfg, checkpoint_dir=member_ckpt_dir,
            criterion_cls=GaussianMemberLoss,
        )
        trainer.train()
        print(f"[train_ensemble] member {i} done -> {member_ckpt_dir}/best_model.pth")

    print(f"\n[train_ensemble] all {args.n_members} members done.")
    print(f"[train_ensemble] load with STGATEnsemble.from_checkpoints(model_cfg, "
          f"[f'{args.out}/member_{{i}}/best_model.pth' for i in range({args.n_members})])")


if __name__ == '__main__':
    main()
