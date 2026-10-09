"""Per-window traces for developing the pose-error estimator offline:
consistency gaps, speed, LiDAR-map health, Layer 1 evidence, and (fault
trials) AWSIM ground-truth pose error. Nominal calibration trials too, for
Q / R from nominal data. Writes <ANALYSIS_ROOT>/layer2_envelope/pose_traces.pkl.

  STGAT_GRAPH_FRAME=ego_v6 python3 experiments/scripts/extract_pose_error_traces.py
"""
import os
import pickle
import sys
from types import SimpleNamespace

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(__file__))
import layer2_envelope as LE  # noqa: E402
L2, cfg = LE.L2, LE.cfg


def main():
    out_dir = os.path.join(cfg.ANALYSIS_ROOT, 'layer2_envelope')
    args = SimpleNamespace(model=cfg.MODEL_CONFIG['model_path'], batch=256, output_dir=out_dir, no_cache=False,
                           kappa_q=0.99, max_episodes_per_hour=6.0, seed=20261007)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = L2._load_model(args.model, device)
    map_data = L2.margin_lib.load_map(cfg.MAP_FILE)
    drivable = L2.margin_lib.DrivableArea(map_data)
    pool = LE.build_pool(model, device, args)
    def raw(seqs):
        g = np.array([s['past'][-1]['consistency'] for s in seqs], dtype=float)
        yaw = np.array([s['past'][-1]['yaw'] for s in seqs], dtype=float)
        return g, yaw
    g_all, yaw_all = raw(pool['seqs'])
    nominal = []
    for tid in np.unique(pool['trial']):
        m = pool['trial'] == tid
        nominal.append({'trial': int(tid), 't': pool['t_all'][m], 'e': pool['e'][m], 'v': pool['v'][m],
                        'g_raw': g_all[m], 'yaw': yaw_all[m],
                        'lq': pool['lq'][m], 'D': pool['ev']['D'][m], 'u': pool['ev']['u'][m]})
    from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor
    from st_gat.pipeline.sequence_builder import SequenceBuilder
    builder = SequenceBuilder(MapProcessor(cfg.MAP_FILE).map_data, route=[])
    faults = []
    camps = sorted(c for c in os.listdir(L2.DATA_DIR) if c.startswith('imu'))
    for run_dir in L2._fault_trials(camps):
        print(run_dir, flush=True)
        tr = LE.process_trial(run_dir, model, device, pool, builder, args)
        if tr is None:
            continue
        g = LE.ground_truth(tr, drivable)
        g_raw, yaw = raw(tr['seqs'])
        faults.append({'trial': tr['rel'], 'g_raw': g_raw, 'yaw': yaw, 't': tr['t'], 't0': tr['t0'], 'e': tr['e'], 'v': tr['v'],
                       'lq': LE.window_locq(tr['seqs']), 'D': tr['ev']['D'], 'u': tr['ev']['u'],
                       'Psi_now': tr['ev']['Psi'], 'Y_now': tr['ev']['Y'], 'X_now': tr['ev']['X'],
                       'fault_windows': tr['fault_windows'], 'departure': g['departure'],
                       'true_psi': g['true_psi'], 'true_lat': g['true_lat'],
                       'gt': tr['gt'], 'est': tr['est']})
    with open(os.path.join(out_dir, 'pose_traces.pkl'), 'wb') as f:
        pickle.dump({'cal': pool['cal'], 'nominal': nominal, 'faults': faults}, f)
    print('saved', len(nominal), 'nominal,', len(faults), 'fault trials')


if __name__ == '__main__':
    main()
