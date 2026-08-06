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
  - Accepts both old-format sequences (no 'uncertainty' key) and new-format
    sequences, so the dataset can be used even with partially processed data.
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
        self.sequences = []
        pkl_files = sorted(f for f in os.listdir(data_folder) if f.endswith('.pkl'))
        for fname in pkl_files:
            fpath = os.path.join(data_folder, fname)
            with open(fpath, 'rb') as f:
                self.sequences.extend(pickle.load(f))
        print(f"[dataset] Loaded {len(self.sequences)} sequences from {len(pkl_files)} files")

    def __len__(self) -> int:
        return len(self.sequences)

    def __getitem__(self, idx: int):
        seq = self.sequences[idx]
        past_t   = self._build_feature_tensors(seq['past'])
        future_t = self._build_feature_tensors(seq['future'])
        graph_t  = self._build_graph_tensors(seq['graph'])
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
            buf['traffic_light_color'].append([float(step.get('traffic_light_color', 0.0))])
            buf['traffic_light_confidence'].append([float(step.get('traffic_light_confidence', 0.0))])
            buf['traffic_light_discrepancy'].append([float(step.get('traffic_light_discrepancy', 0.0))])
            buf['has_adjacent_lane'].append([float(step.get('has_adjacent_lane', 0.0))])
            buf['uncertainty'].append(step.get('uncertainty', [0.0, 0.0]))
            buf['objects_set'].append(step['objects_set'])
            buf['objects_mask'].append(step['objects_mask'])

        return {k: torch.tensor(np.asarray(v), dtype=torch.float32) for k, v in buf.items()}

    @staticmethod
    def _build_graph_tensors(G) -> dict:
        """Convert a networkx.Graph → node_features, adjacency matrix, and
        node_mask tensors. node_mask added 2026-08-05: graphs are now
        radius-scoped (config.py's GRAPH_RADIUS_M) rather than always padded
        to exactly _MAX_GRAPH_NODES real nodes, so GraphEncoder needs to know
        which rows are real vs. zero-padding — see model.py's GraphEncoder
        for the attention-pooling mask this feeds."""
        node_features = torch.zeros((_MAX_GRAPH_NODES, _NODE_FEATURES), dtype=torch.float32)
        node_mask     = torch.zeros((_MAX_GRAPH_NODES,), dtype=torch.float32)
        for node_id, data in G.nodes(data=True):
            if node_id < _MAX_GRAPH_NODES:
                node_features[node_id] = torch.tensor([
                    float(data['x']),
                    float(data['y']),
                    float(data.get('traffic_light_detection_node', 0)),
                    float(data.get('path_node', 0)),
                ], dtype=torch.float32)
                node_mask[node_id] = 1.0

        raw_adj = nx.to_numpy_array(G)
        n = min(raw_adj.shape[0], _MAX_GRAPH_NODES)
        adj = torch.zeros((_MAX_GRAPH_NODES, _MAX_GRAPH_NODES), dtype=torch.float32)
        adj[:n, :n] = torch.tensor(raw_adj[:n, :n], dtype=torch.float32)
        adj_t = adj

        return {'node_features': node_features, 'adj_matrix': adj_t, 'node_mask': node_mask}
