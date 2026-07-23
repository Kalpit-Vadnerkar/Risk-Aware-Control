"""
STGAT — Spatial-Temporal Graph Attention Network (RISE edition).

Architecture overview:
  1. GraphEncoder  — 2-layer GCN with LayerNorm, mean-pool → d_g context vector
  2. InputNorm     — BatchNorm1d across (batch × time) for each input feature
  3. InputProj     — Linear(F → d_m) → GELU → LayerNorm
  4. GraphFusion   — learned linear maps graph context into d_m, adds to each timestep
  5. TemporalAttn  — nn.TransformerEncoderLayer (nhead=4, ffn=2×d_m) for global mixing
  6. LSTM          — 2-layer LSTM (d_m → d_h) for recurrent sequential dynamics
  7. OutputHeads   — 6 small linear heads sharing the final LSTM hidden state

Key improvements over the T-ITS paper's GraphAttentionLSTM:
  - Single LSTM instead of 6 identical ones (5× fewer parameters)
  - BatchNorm on inputs handles heterogeneous feature scales (including new
    position_uncertainty which sits near 0.006 vs position in [0, 1])
  - LayerNorm after each GCN layer stabilises graph feature magnitudes
  - Built-in gradient clipping in the Trainer (not in this module)
  - softplus(var) + VAR_FLOOR in loss prevents NLL from collapsing to -inf

Total parameters: ~1.2M  (vs ~9M original)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


# Variance floor applied when computing NLL loss (not in model — see loss.py)
VAR_FLOOR = 1e-4


# ── Graph Encoder ──────────────────────────────────────────────────────────

class _GCNLayer(nn.Module):
    """Single graph convolution: H' = A · H · W, followed by LayerNorm."""

    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.weight = nn.Parameter(torch.empty(in_dim, out_dim))
        self.norm   = nn.LayerNorm(out_dim)
        nn.init.xavier_uniform_(self.weight)

    def forward(self, h: torch.Tensor, adj: torch.Tensor) -> torch.Tensor:
        # h: (N, in_dim)   adj: (N, N)
        return self.norm(adj @ h @ self.weight)


class GraphEncoder(nn.Module):
    """
    Two GCN layers → mean-pool over nodes → d_g-dim context vector.
    Operates on a single graph at a time; called inside a batch loop.
    """

    def __init__(self, node_feat_dim: int = 4, d_g: int = 128, dropout: float = 0.15):
        super().__init__()
        self.gc1     = _GCNLayer(node_feat_dim, d_g)
        self.gc2     = _GCNLayer(d_g, d_g)
        self.dropout = nn.Dropout(dropout)

    def forward(self, node_features: torch.Tensor, adj: torch.Tensor) -> torch.Tensor:
        """
        node_features: (B, N, node_feat_dim)
        adj:           (B, N, N)
        returns:       (B, d_g)
        """
        B = node_features.size(0)
        ctx_list = []
        for i in range(B):
            h = F.gelu(self.gc1(node_features[i], adj[i]))
            h = self.dropout(h)
            h = F.gelu(self.gc2(h, adj[i]))
            h = self.dropout(h)
            ctx_list.append(h.mean(dim=0))   # mean-pool over nodes
        return torch.stack(ctx_list)          # (B, d_g)


# ── Main model ─────────────────────────────────────────────────────────────

class STGAT(nn.Module):
    """
    Spatial-Temporal Graph Attention Network for probabilistic trajectory prediction.

    Input dict keys expected in `x`:
        position (B, T, 2), velocity (B, T, 2), steering (B, T, 1),
        acceleration (B, T, 1), object_distance (B, T, 1),
        traffic_light_detected (B, T, 1), uncertainty (B, T, 2)

    Output dict:
        position_mean/var  (B, T_out, 2)
        velocity_mean/var  (B, T_out, 2)
        steering_mean/var  (B, T_out)
        acceleration_mean/var (B, T_out)
        object_distance    (B, T_out)    — sigmoid-bounded
        traffic_light_detected (B, T_out) — sigmoid-bounded
    """

    def __init__(self, config: dict):
        super().__init__()

        self.T_in  = config['input_seq_len']
        self.T_out = config['output_seq_len']

        d_m       = config.get('d_model', 128)        # model / attention dim
        d_g       = config.get('d_graph', 128)        # graph encoder output dim
        d_h       = config.get('hidden_size', 128)    # LSTM hidden dim
        n_layers  = config.get('num_layers', 2)
        dropout   = config.get('dropout_rate', 0.15)
        nhead     = config.get('nhead', 4)
        node_fdim = config['graph_sizes']['node_features']

        F_total = sum(config['feature_sizes'].values())   # 14 with RISE features + discrepancy

        self.feature_keys = list(config['feature_sizes'].keys())

        # ── Graph branch ────────────────────────────────────────────────────
        self.graph_encoder = GraphEncoder(node_fdim, d_g, dropout)
        self.graph_proj    = nn.Sequential(
            nn.Linear(d_g, d_m),
            nn.GELU(),
            nn.LayerNorm(d_m),
        )

        # ── Vehicle feature branch ──────────────────────────────────────────
        # Normalize raw inputs: (B*T, F) — handles different feature magnitudes
        self.input_bn   = nn.BatchNorm1d(F_total)
        self.input_proj = nn.Sequential(
            nn.Linear(F_total, d_m),
            nn.GELU(),
            nn.LayerNorm(d_m),
        )

        # ── Temporal attention (global over T=30 timesteps) ─────────────────
        self.attn_block = nn.TransformerEncoderLayer(
            d_model         = d_m,
            nhead           = nhead,
            dim_feedforward = d_m * 2,   # 2× expansion — enough for 30-step seq
            dropout         = dropout,
            activation      = 'gelu',
            batch_first     = True,
            norm_first      = True,      # Pre-LN: more stable gradients
        )

        # ── Recurrent encoder ───────────────────────────────────────────────
        self.lstm = nn.LSTM(
            input_size   = d_m,
            hidden_size  = d_h,
            num_layers   = n_layers,
            dropout      = dropout if n_layers > 1 else 0.0,
            batch_first  = True,
        )
        self.lstm_drop = nn.Dropout(dropout)

        # ── Output heads (all share the single LSTM's final hidden state) ────
        # Each head maps d_h → output_dim * T_out then reshapes
        self.head_position    = nn.Linear(d_h, 4  * self.T_out)   # mean(2) + var(2)
        self.head_velocity    = nn.Linear(d_h, 4  * self.T_out)
        self.head_steering    = nn.Linear(d_h, 2  * self.T_out)   # mean(1) + var(1)
        self.head_accel       = nn.Linear(d_h, 2  * self.T_out)
        self.head_obj_dist    = nn.Linear(d_h, self.T_out)
        self.head_traffic     = nn.Linear(d_h, self.T_out)

        self._init_weights()

    def _init_weights(self):
        for name, p in self.named_parameters():
            if 'weight_ih' in name:
                nn.init.xavier_uniform_(p)
            elif 'weight_hh' in name:
                nn.init.orthogonal_(p)
            elif 'bias' in name and 'lstm' in name.lower():
                nn.init.zeros_(p)
                # LSTM forget gate bias = 1.0 → better gradient flow early in training
                n = p.size(0) // 4
                p.data[n:2*n].fill_(1.0)

    def forward(self, x: dict, graph: dict) -> dict:
        node_features = graph['node_features']   # (B, N, node_fdim)
        adj_matrix    = graph['adj_matrix']      # (B, N, N)
        B = node_features.size(0)

        # ── 1. Ensure all features are (B, T, dim) ──────────────────────────
        parts = []
        for key in self.feature_keys:
            t = x[key]
            if t.dim() == 2:
                t = t.unsqueeze(-1)
            parts.append(t)
        x_cat = torch.cat(parts, dim=-1)   # (B, T, F_total)

        # ── 2. Input normalisation ───────────────────────────────────────────
        T = x_cat.size(1)
        x_flat   = x_cat.view(B * T, -1)
        x_flat   = self.input_bn(x_flat)
        x_normed = x_flat.view(B, T, -1)

        # ── 3. Project vehicle features ──────────────────────────────────────
        seq = self.input_proj(x_normed)    # (B, T, d_m)

        # ── 4. Graph context — additive fusion over the time axis ────────────
        graph_ctx  = self.graph_encoder(node_features, adj_matrix)  # (B, d_g)
        graph_ctx  = self.graph_proj(graph_ctx)                      # (B, d_m)
        seq = seq + graph_ctx.unsqueeze(1)                           # broadcast over T

        # ── 5. Self-attention over the 30-step window ────────────────────────
        seq = self.attn_block(seq)         # (B, T, d_m)

        # ── 6. Recurrent encoding ────────────────────────────────────────────
        lstm_out, _ = self.lstm(seq)       # (B, T, d_h)
        h_last = self.lstm_drop(lstm_out[:, -1])   # (B, d_h) — last hidden state

        # ── 7. Output heads ──────────────────────────────────────────────────
        T_o = self.T_out

        pos   = self.head_position(h_last).view(B, T_o, 4)
        vel   = self.head_velocity(h_last).view(B, T_o, 4)
        steer = self.head_steering(h_last).view(B, T_o, 2)
        accel = self.head_accel(h_last).view(B, T_o, 2)
        obj   = self.head_obj_dist(h_last).view(B, T_o)
        tl    = self.head_traffic(h_last).view(B, T_o)

        return {
            'position_mean':    pos[..., :2],
            'position_var':     F.softplus(pos[..., 2:]) + VAR_FLOOR,
            'velocity_mean':    vel[..., :2],
            'velocity_var':     F.softplus(vel[..., 2:]) + VAR_FLOOR,
            'steering_mean':    steer[..., 0],
            'steering_var':     F.softplus(steer[..., 1]) + VAR_FLOOR,
            'acceleration_mean': accel[..., 0],
            'acceleration_var':  F.softplus(accel[..., 1]) + VAR_FLOOR,
            'object_distance':        torch.sigmoid(obj),
            'traffic_light_detected': torch.sigmoid(tl),
        }

    def count_parameters(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
