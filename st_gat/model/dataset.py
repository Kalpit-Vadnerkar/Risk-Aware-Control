"""
TrajectoryDataset for STGAT (RISE edition).

Changes from the T-ITS reference TrajectoryDataset:
  - Adds 'uncertainty' key (x_var, y_var from EKF — already scaled to [0,1]
    by the pipeline, so no additional scaling applied)
  - Removes the per-feature scaling factors (position_scaling_factor etc.)
    All features come pre-normalised to [0, 1] from sequence_builder.py.
    The model's BatchNorm1d layer handles any residual scale differences.
  - Graph node features stored without the 10× position multiplier for the
    same reason — GCN's LayerNorm handles scale.
  - Old-vs-new-format tolerance REMOVED 2026-08-07 (audit finding): used to
    read every TL/uncertainty/adjacent-lane key via step.get(key, 0.0), so a
    stale pre-redesign sequence would silently load with those features
    zeroed out instead of erroring. Now that config.py's schema-versioning
    check (see check_schema_manifest) refuses to load a cache built under a
    different schema at all, this fallback no longer serves its original
    purpose and was actively hiding the exact bug class it was flagged for
    -- these keys are now read with direct (KeyError-on-miss) access, same
    as position/velocity/steering.
"""

import os
import pickle

import networkx as nx
import numpy as np
import torch
from torch.utils.data import Dataset

from ..pipeline import config as _cfg

# Graph constants -- imported from pipeline config instead of a separately
# hardcoded copy (fixed 2026-08-05: this file used to hardcode its own
# _MAX_GRAPH_NODES=150/_NODE_FEATURES=4, independent of config.py's
# MAX_GRAPH_NODES/NODE_FEATURES, with only a "kept in sync" comment holding
# the two in agreement -- exactly the duplicated-independently-maintained-
# copy pattern that let the retired st_gat/infer.py drift stale. Caught when
# raising MAX_GRAPH_NODES for the radius-gated graph redesign (see
# docs/research_notes/ablation_study_2026.md §7/§10) would otherwise have
# required remembering to also update this file by hand.)
_MAX_GRAPH_NODES = _cfg.MAX_GRAPH_NODES
_NODE_FEATURES   = _cfg.NODE_FEATURES


class TrajectoryDataset(Dataset):
    """
    Loads .pkl files from a folder. Each file is a list of sequence dicts
    produced by sequence_builder.SequenceBuilder.build().

    Each sequence dict has:
        'past':   list of T_in  processed timestep dicts
        'future': list of T_out processed timestep dicts
        'graph':  networkx.Graph
        'graph_bounds': [x_min, x_max, y_min, y_max]  (not used by model, kept for analysis)

    Each processed timestep dict has:
        position (list[2]), velocity (list[2]), steering (float),
        acceleration (float),
        traffic_light_color (float), traffic_light_confidence (float),
        traffic_light_discrepancy (int/float), has_adjacent_lane (float),
        uncertainty (list[2]),
        objects_set ((K, OBJECT_FEATURE_DIM) array), objects_mask ((K,) array)
        — added 2026-08-02, replacing object_distance/closest_object_velocity
        (see sequence_builder.py's _build_object_set).

        traffic_light_color/traffic_light_confidence replace the old
        traffic_light_state (color*confidence collapsed into one scalar) and
        traffic_light_detected (a manufactured "map expects a TL here" proxy,
        retired as an output feature 2026-08-05 — see config.py's
        FEATURE_SIZES doc and GraphBuilder.py's _add_traffic_light_nodes).
    """

    _SCALAR_KEYS = (
        'steering', 'acceleration', 'traffic_light_color',
        'traffic_light_confidence', 'traffic_light_discrepancy', 'has_adjacent_lane',
    )
    _VECTOR_KEYS = ('position', 'velocity', 'uncertainty')

    def __init__(self, data_folder: str):
        # Schema-versioning guard (2026-08-07, see config.py's
        # check_schema_manifest docstring) -- data_folder (TRAIN_DIR/CAL_DIR)
        # is a subdir of SEQUENCES_DIR, which is where run_pipeline.py's
        # assemble_splits() writes the manifest; check the PARENT dir, not
        # data_folder itself, since that's the manifest's actual location.
        # write_if_missing=False deliberately: a cache with .pkl content but
        # no manifest predates this fix and must not be silently trusted,
        # even by a caller that isn't run_pipeline.py itself.
        _cfg.check_schema_manifest(os.path.dirname(os.path.normpath(data_folder)), write_if_missing=False)
        self.sequences = []
        pkl_files = sorted(f for f in os.listdir(data_folder) if f.endswith('.pkl'))
        for fname in pkl_files:
            fpath = os.path.join(data_folder, fname)
            with open(fpath, 'rb') as f:
                self.sequences.extend(pickle.load(f))
        print(f"[dataset] Loaded {len(self.sequences)} sequences from {len(pkl_files)} files")
        # Warm the per-graph array cache once, in the parent process, so
        # DataLoader workers inherit it (fork) instead of each rebuilding it.
        # ~30 consecutive windows share one graph object (sequence_builder's
        # rebuild cache), so this is ~1/30th of the per-sample work.
        for g in {id(s['graph']): s['graph'] for s in self.sequences}.values():
            _graph_arrays(g)

    def __len__(self) -> int:
        return len(self.sequences)

    def __getitem__(self, idx: int):
        seq = self.sequences[idx]
        past_t   = self._build_feature_tensors(seq['past'])
        future_t = self._build_feature_tensors(seq['future'])
        graph_t  = self._build_graph_tensors(seq['graph'], seq.get('position_ref'), seq.get('heading_ref'),
                                            seq.get('route_ahead'))
        return past_t, future_t, graph_t, seq['graph_bounds']

    # ── Internal helpers ───────────────────────────────────────────────────
    # staticmethods (not instance methods) so st_gat/residuals.py can reuse
    # them directly instead of re-implementing its own copy — a duplicated,
    # independently-maintained copy of this logic is exactly what let the
    # retired st_gat/infer.py drift out of sync with cfg.FEATURE_SIZES
    # (docs/stgat_pipeline_plan.md §1.12 / TODO.md Phase 1.3).

    @staticmethod
    def _build_feature_tensors(steps: list) -> dict:
        """Convert a list of timestep dicts → dict of float32 tensors."""
        buf = {
            'position':                 [],
            'velocity':                 [],
            'steering':                 [],
            'acceleration':             [],
            'traffic_light_color':      [],
            'traffic_light_confidence': [],
            'traffic_light_discrepancy': [],
            'has_adjacent_lane':        [],
            'uncertainty':              [],
            'objects_set':              [],
            'objects_mask':             [],
        }

        for step in steps:
            buf['position'].append(step['position'])
            buf['velocity'].append(step['velocity'])
            buf['steering'].append([step['steering']])
            buf['acceleration'].append([step['acceleration']])
            buf['traffic_light_color'].append([float(step['traffic_light_color'])])
            buf['traffic_light_confidence'].append([float(step['traffic_light_confidence'])])
            buf['traffic_light_discrepancy'].append([float(step['traffic_light_discrepancy'])])
            buf['has_adjacent_lane'].append([float(step['has_adjacent_lane'])])
            buf['uncertainty'].append(step['uncertainty'])
            buf['objects_set'].append(step['objects_set'])
            buf['objects_mask'].append(step['objects_mask'])
        if steps and 'yaw' in steps[0]:   # ego_v2 schema only -- rotation input, not a model feature
            buf['yaw'] = [[float(step['yaw'])] for step in steps]

        return {k: torch.tensor(np.asarray(v), dtype=torch.float32) for k, v in buf.items()}

    @staticmethod
    def _build_graph_tensors(G, ref=None, yaw=None, route_ahead=None) -> dict:
        """Convert a networkx.Graph → node_features, adjacency matrix, and
        node_mask tensors. node_mask added 2026-08-05: graphs are now
        radius-scoped (config.py's GRAPH_RADIUS_M) rather than always padded
        to exactly _MAX_GRAPH_NODES real nodes, so GraphEncoder needs to know
        which rows are real vs. zero-padding — see model.py's GraphEncoder
        for the attention-pooling mask this feeds.

        Vectorized 2026-10-05 (was a per-node Python loop + a fresh
        nx.to_numpy_array per sample, ~9ms/sample -- the training-time
        bottleneck): per-graph arrays come from _graph_arrays()'s cache, and
        the adjacency is uint8 (0/1; the model casts on device) -- a float32
        1024x1024 matrix was 4MB per sample to collate and copy."""
        a = _graph_arrays(G)
        n = a['n']
        node_features = np.zeros((_MAX_GRAPH_NODES, _NODE_FEATURES), dtype=np.float32)
        if _cfg.EGO_FRAME:
            # ego_v2 (config.py's GRAPH_FRAME note): raw-metre nodes -> this
            # window's ego frame (x forward, y left), fixed metric scale.
            if ref is None or yaw is None:
                raise ValueError("ego_v2 graphs need position_ref and heading_ref "
                                 "(_build_graph_tensors(seq['graph'], seq['position_ref'], seq['heading_ref']))")
            c, s_ = np.cos(-yaw), np.sin(-yaw)
            d = a['xy'] - np.asarray(ref, dtype=np.float64)[None, :]
            u = a['dir']
            node_features[:n, 0] = (c * d[:, 0] - s_ * d[:, 1]) / _cfg.GRAPH_NODE_SCALE_M
            node_features[:n, 1] = (s_ * d[:, 0] + c * d[:, 1]) / _cfg.GRAPH_NODE_SCALE_M
            node_features[:n, 2] = a['tl']
            node_features[:n, 3] = a['path']
            node_features[:n, 4] = c * u[:, 0] - s_ * u[:, 1]
            node_features[:n, 5] = s_ * u[:, 0] + c * u[:, 1]
        else:
            node_features[:n, 0:2] = a['xy']
            node_features[:n, 2] = a['tl']
            node_features[:n, 3] = a['path']
        node_mask = np.zeros((_MAX_GRAPH_NODES,), dtype=np.float32)
        node_mask[:n] = 1.0
        adj = np.zeros((_MAX_GRAPH_NODES, _MAX_GRAPH_NODES), dtype=np.uint8)
        adj[a['rows'], a['cols']] = 1
        out = {'node_features': torch.from_numpy(node_features),
               'adj_matrix': torch.from_numpy(adj),
               'node_mask': torch.from_numpy(node_mask)}
        if _cfg.ROUTE_AHEAD:
            # ego_v3: route-ahead polyline in the same ego frame/scale as the
            # nodes; route_valid=0 (and zeros) when the bag had no route.
            ra = np.zeros((_cfg.ROUTE_AHEAD_POINTS, 2), dtype=np.float32)
            if route_ahead is not None:
                d = np.asarray(route_ahead, dtype=np.float64) - np.asarray(ref, dtype=np.float64)[None, :]
                ra[:, 0] = (c * d[:, 0] - s_ * d[:, 1]) / _cfg.GRAPH_NODE_SCALE_M
                ra[:, 1] = (s_ * d[:, 0] + c * d[:, 1]) / _cfg.GRAPH_NODE_SCALE_M
            out['route_ahead'] = torch.from_numpy(ra)
            out['route_valid'] = torch.tensor([float(route_ahead is not None)])
        return out


def _graph_arrays(G) -> dict:
    """Per-graph node arrays + edge list, computed once and cached on the
    graph object (G.graph['_arrays']; graphs are shared by every window in
    a rebuild period and never otherwise mutated). Only node ids below
    _MAX_GRAPH_NODES are kept, as before; adjacency entries match
    nx.to_numpy_array's 0/1 values for this simple, unweighted graph."""
    cached = G.graph.get('_arrays')
    if cached is not None:
        return cached
    nodes = [(nid, d) for nid, d in G.nodes(data=True) if nid < _MAX_GRAPH_NODES]
    n = (max(nid for nid, _ in nodes) + 1) if nodes else 0
    xy = np.zeros((n, 2)); dr = np.zeros((n, 2)); tl = np.zeros(n); path = np.zeros(n)
    for nid, d in nodes:
        xy[nid] = (float(d['x']), float(d['y']))
        dr[nid] = (float(d.get('dir_x', 0.0)), float(d.get('dir_y', 0.0)))
        tl[nid] = float(d.get('traffic_light_detection_node', 0))
        path[nid] = float(d.get('path_node', 0))
    rows, cols = [], []
    for a_, b_ in G.edges():
        if a_ < _MAX_GRAPH_NODES and b_ < _MAX_GRAPH_NODES:
            rows += [a_, b_]
            cols += [b_, a_]
    out = {'n': n, 'xy': xy, 'dir': dr, 'tl': tl, 'path': path,
           'rows': np.asarray(rows, dtype=np.int64), 'cols': np.asarray(cols, dtype=np.int64)}
    G.graph['_arrays'] = out
    return out
