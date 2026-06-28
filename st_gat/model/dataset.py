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

# Graph constants (kept in sync with pipeline config)
_MAX_GRAPH_NODES = 150
_NODE_FEATURES   = 4     # x, y, traffic_light_detection_node, path_node


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
        acceleration (float), object_distance (float),
        traffic_light_detected (int/float), traffic_light_state (float),
        closest_object_velocity (float), has_adjacent_lane (float),
        uncertainty (list[2])
    """

    _SCALAR_KEYS = (
        'steering', 'acceleration', 'object_distance', 'traffic_light_detected',
        'traffic_light_state', 'closest_object_velocity', 'has_adjacent_lane',
    )
    _VECTOR_KEYS = ('position', 'velocity', 'uncertainty')
    # Keys added after initial pipeline release; default to 0 for old pkl files
    _OPTIONAL_KEYS = ('uncertainty', 'traffic_light_state', 'closest_object_velocity', 'has_adjacent_lane')

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

    def _build_feature_tensors(self, steps: list) -> dict:
        """Convert a list of timestep dicts → dict of float32 tensors."""
        buf = {
            'position':                 [],
            'velocity':                 [],
            'steering':                 [],
            'acceleration':             [],
            'object_distance':          [],
            'traffic_light_detected':   [],
            'traffic_light_state':      [],
            'closest_object_velocity':  [],
            'has_adjacent_lane':        [],
            'uncertainty':              [],
        }

        for step in steps:
            buf['position'].append(step['position'])
            buf['velocity'].append(step['velocity'])
            buf['steering'].append([step['steering']])
            buf['acceleration'].append([step['acceleration']])
            buf['object_distance'].append([step['object_distance']])
            buf['traffic_light_detected'].append([float(step['traffic_light_detected'])])
            buf['traffic_light_state'].append([float(step.get('traffic_light_state', 0.0))])
            buf['closest_object_velocity'].append([float(step.get('closest_object_velocity', 0.0))])
            buf['has_adjacent_lane'].append([float(step.get('has_adjacent_lane', 0.0))])
            buf['uncertainty'].append(step.get('uncertainty', [0.0, 0.0]))

        return {k: torch.tensor(v, dtype=torch.float32) for k, v in buf.items()}

    def _build_graph_tensors(self, G) -> dict:
        """Convert a networkx.Graph → node_features and adjacency matrix tensors."""
        node_features = torch.zeros((_MAX_GRAPH_NODES, _NODE_FEATURES), dtype=torch.float32)
        for node_id, data in G.nodes(data=True):
            if node_id < _MAX_GRAPH_NODES:
                node_features[node_id] = torch.tensor([
                    float(data['x']),
                    float(data['y']),
                    float(data.get('traffic_light_detection_node', 0)),
                    float(data.get('path_node', 0)),
                ], dtype=torch.float32)

        raw_adj = nx.to_numpy_array(G)
        n = min(raw_adj.shape[0], _MAX_GRAPH_NODES)
        adj = torch.zeros((_MAX_GRAPH_NODES, _MAX_GRAPH_NODES), dtype=torch.float32)
        adj[:n, :n] = torch.tensor(raw_adj[:n, :n], dtype=torch.float32)
        adj_t = adj

        return {'node_features': node_features, 'adj_matrix': adj_t}
