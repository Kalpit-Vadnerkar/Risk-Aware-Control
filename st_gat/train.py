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
import sys

import torch
from torch.utils.data import DataLoader, WeightedRandomSampler

from st_gat.pipeline import config as cfg
from st_gat.model import STGAT, TrajectoryDataset, Trainer
from st_gat.model import loss as loss_module

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'experiments', 'lib'))
import scenario_zones  # noqa: E402


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
        'freeze_trunk_phase2': args.freeze_trunk_phase2,
        'resume_warmup_from': args.resume_warmup_from,
        'warmup_only': args.warmup_only,
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
    parser.add_argument('--no-task-weighting', action='store_true',
                        help='Disable Kendall et al. learned per-task loss weighting in Phase 2, '
                             'use fixed DEFAULT_WEIGHTS instead (2026-08-19 -- see loss.py CombinedLoss '
                             'docstring: that mechanism has caused one documented divergence and has '
                             'never been validated to help)')
    parser.add_argument('--var-reg-weight', type=float, default=None,
                        help='Override loss.py VAR_REG_WEIGHT (default: module constant, currently '
                             '0.1, never swept -- see loss.py docstring point 3). Monkeypatched onto '
                             'the loss module before training, per that module\'s own documented '
                             'intent for a sweep script to do this.')
    parser.add_argument('--freeze-trunk-phase2', action='store_true',
                        help='Freeze the shared trunk (graph/temporal encoder -> h_last) for Phase 2, '
                             'training only each head\'s own MLP + horizon embedding (2026-08-20 -- '
                             'see model.py STGAT.freeze_trunk docstring: tests whether the trunk '
                             'continuing to adapt under Phase 2, driven by whichever head\'s gradient '
                             'is largest, is what degrades validation NLL for every head in lockstep)')
    parser.add_argument('--dof-reg-weight', type=float, default=None,
                        help='Override loss.py DOF_REG_WEIGHT (default: module constant, currently '
                             '0.05). Only applies to DOF_REG_KEYS (position/velocity/steering/'
                             'acceleration) -- deliberately not the TL heads, see loss.py docstring.')
    parser.add_argument('--dof-reg-target', type=float, default=None,
                        help='Override loss.py DOF_REG_TARGET (default: module constant, currently 5.0)')
    parser.add_argument('--resume-warmup-from', type=str, default=None,
                        help='Skip Phase 1 (mean-only warmup) entirely and load weights from this '
                             'mean_warmup.pth instead (2026-08-20) -- valid across any Phase-2-only '
                             'config change (VAR_REG_WEIGHT/DOF_REG_WEIGHT/--no-task-weighting/'
                             '--freeze-trunk-phase2 all only affect Phase 2), saves ~90min/run. Only '
                             'valid if model.py has not changed shape since that checkpoint was saved.')
    parser.add_argument('--warmup-only', action='store_true',
                        help='Stop after Phase 1 (mean-only warmup), no Phase 2 at all -- for training '
                             'independent point-predictor-only members to check EPISTEMIC (cross-member '
                             'disagreement) horizon-widening rather than aleatoric (self-reported '
                             'scale/dof). 2026-08-20.')
    parser.add_argument('--zone-weighted-sampling', action='store_true',
                        help='Oversample training windows near turn/intersection zones instead of uniform '
                             'shuffling (2026-08-24) -- fixes the turn-anticipation gap found via '
                             'diagnose_turn_learning.py/audit_minority_scenarios.py at the model level '
                             'instead of only widening the calibrated interval there (which would trade '
                             'away detection sensitivity exactly where fault_injector.py targets faults). '
                             'See experiments/lib/scenario_zones.py for the weighting formula.')
    parser.add_argument('--turn-boost', type=float, default=3.0,
                        help='Extra sample weight (on top of 1.0 baseline) for a window whose OWN actual '
                             'future turn severity hits --turn-cap-deg or more; scales linearly below that.')
    parser.add_argument('--tl-boost', type=float, default=2.0,
                        help='Extra flat sample weight for a window starting within a real TL/intersection zone.')
    parser.add_argument('--turn-cap-deg', type=float, default=20.0,
                        help='Turn severity (degrees of heading change over the horizon) at which --turn-boost '
                             'reaches its full value.')
    args = parser.parse_args()

    if args.var_reg_weight is not None:
        print(f"[train] Overriding VAR_REG_WEIGHT: {loss_module.VAR_REG_WEIGHT} -> {args.var_reg_weight}")
        loss_module.VAR_REG_WEIGHT = args.var_reg_weight
    if args.dof_reg_weight is not None:
        print(f"[train] Overriding DOF_REG_WEIGHT: {loss_module.DOF_REG_WEIGHT} -> {args.dof_reg_weight}")
        loss_module.DOF_REG_WEIGHT = args.dof_reg_weight
    if args.dof_reg_target is not None:
        print(f"[train] Overriding DOF_REG_TARGET: {loss_module.DOF_REG_TARGET} -> {args.dof_reg_target}")
        loss_module.DOF_REG_TARGET = args.dof_reg_target

    model_cfg = build_model_config(args)

    print(f"[train] device: {model_cfg['device']}")
    print(f"[train] train dir: {cfg.TRAIN_DIR}")
    print(f"[train] cal dir:   {cfg.CAL_DIR}")

    train_ds = TrajectoryDataset(cfg.TRAIN_DIR)
    val_ds   = TrajectoryDataset(cfg.CAL_DIR)

    # Deliberately NOT using persistent_workers (tried 2026-08-24, reverted):
    # caused two separate systemd-oomd kills (12 workers: OOM by epoch 2; 8
    # workers + explicit MemoryMax cap: OOM by epoch 5) despite thread-count
    # capping. Root cause: CPython's fork-based worker processes lose
    # copy-on-write sharing gradually as refcounts on touched objects get
    # bumped (a well-known fork+refcounting gotcha) -- persistent_workers
    # lets that accumulate WITHIN each worker across every epoch it stays
    # alive, instead of resetting via a fresh fork every epoch (the default,
    # non-persistent behavior every previously-working run -- e.g.
    # h30_30_pointpred_v1's mean_warmup.pth -- always used). The modest
    # per-epoch respawn overhead this brings back is a much better trade
    # than periodic OOM kills.
    if args.zone_weighted_sampling:
        print(f"[train] Computing zone-weighted sample weights "
              f"(turn_boost={args.turn_boost}, tl_boost={args.tl_boost}, turn_cap_deg={args.turn_cap_deg})...")
        weights = scenario_zones.compute_train_sample_weights(
            train_ds, cfg, turn_boost=args.turn_boost, tl_boost=args.tl_boost, turn_cap_deg=args.turn_cap_deg)
        print(f"[train] Sample weight stats: min={weights.min():.2f} max={weights.max():.2f} "
              f"mean={weights.mean():.2f} (baseline 1.0)")
        sampler = WeightedRandomSampler(weights, num_samples=len(train_ds), replacement=True)
        train_loader = DataLoader(
            train_ds, batch_size=args.batch, sampler=sampler,
            num_workers=args.workers, pin_memory=True, drop_last=True,
        )
    else:
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
        model            = model,
        train_loader     = train_loader,
        val_loader       = val_loader,
        config           = model_cfg,
        checkpoint_dir   = args.out,
        criterion_kwargs = {'use_task_weighting': not args.no_task_weighting},
    )

    trained_model = trainer.train()

    # Only copy to the shared model_path (st_gat/models/<horizon>/st_gat_rise.pth
    # -- what analysis scripts default to loading) when --out is the default
    # checkpoint dir, i.e. this run is meant to BE the official model. Fixed
    # 2026-08-20: every one-off experimental run (--out pointing at a named
    # test dir) was unconditionally overwriting this shared path regardless,
    # silently clobbering it for anything relying on the default --model
    # argument in check_calibration.py/plot_calibration_diagrams.py/etc.
    # Found live: st_gat_rise.pth ended up holding an unfinished experimental
    # checkpoint after several such runs; restored via `git checkout` since
    # it's tracked. Experimental runs' real output is args.out's checkpoint
    # dir -- that's what --model should point at explicitly.
    if args.out == cfg.CHECKPOINT_DIR:
        final_path = model_cfg['model_path']
        os.makedirs(os.path.dirname(final_path), exist_ok=True)
        torch.save(trained_model.state_dict(), final_path)
        print(f"[train] Final model saved to {final_path}")
    else:
        print(f"[train] --out is not the default checkpoint dir -- skipping the "
              f"shared model_path save (experimental run; use --model "
              f"{os.path.join(args.out, 'best_model.pth')} explicitly for analysis scripts)")


if __name__ == '__main__':
    main()
