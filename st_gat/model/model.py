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
  - VAR_FLOOR lowered 1e-4 -> 1e-8 (2026-08-02): one global floor applies to
    every Gaussian-headed feature (position, velocity, steering,
    acceleration, traffic_light_state), but their natural error scales
    differ a lot. Found live: after the position-representation fix,
    real position error is ~0.09m out of a +-100m range (~0.0009 scaled,
    squared ~8e-7) — far below the old 1e-4 floor, so predicted position
    variance was pinned AT the floor (sqrt(1e-4)*100m = 1.0m predicted std,
    exactly the observed value) regardless of how accurate the mean
    prediction became. Velocity/steering/traffic_light_state already sat
    comfortably above 1e-4 (their calibration was fine or improving at that
    floor), so lowering it doesn't force them lower — it just stops being
    the binding constraint for position specifically.

Total parameters: ~1.2M  (vs ~9M original)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


# Variance floor applied when computing NLL loss (not in model — see loss.py)
VAR_FLOOR = 1e-8


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
    Two GCN layers → learned attention-weighted pool over nodes → d_g-dim
    context vector. Operates on a single graph at a time; called inside a
    batch loop.

    Was a uniform mean-pool (fixed 2026-08-05 — see
    docs/research_notes/ablation_study_2026.md §3): even with
    GraphBuilder.build_graph()'s route-aware node selection (§2, also fixed
    2026-08-05) putting route nodes in the graph, a uniform mean over ~150
    nodes still gives each node the same 1/N weight regardless of relevance.
    A learned per-node attention score lets the model itself decide which
    nodes matter for the weighted sum, instead of the architecture forcing
    uniform weight by construction.

    Node mask added 2026-08-05 alongside GraphBuilder's radius-based node
    scoping (config.py's GRAPH_RADIUS_M): graphs are no longer always padded
    to exactly _MAX_GRAPH_NODES real nodes, so the zero-feature padding rows
    need to be masked out of the attention softmax the same way
    ObjectSetEncoder already masks padding objects — before this, mask was
    unnecessary because MIN_GRAPH_NODES == MAX_GRAPH_NODES guaranteed every
    slot was real.
    """

    def __init__(self, node_feat_dim: int = 4, d_g: int = 128, dropout: float = 0.15):
        super().__init__()
        self.gc1        = _GCNLayer(node_feat_dim, d_g)
        self.gc2        = _GCNLayer(d_g, d_g)
        self.dropout    = nn.Dropout(dropout)
        self.attn_score = nn.Linear(d_g, 1)

    def forward(self, node_features: torch.Tensor, adj: torch.Tensor,
                mask: torch.Tensor = None) -> torch.Tensor:
        """
        node_features: (B, N, node_feat_dim)
        adj:           (B, N, N)
        mask:          (B, N) — 1.0 for real nodes, 0.0 for padding. None
                       treats every node as real (backward-compatible with
                       any caller that hasn't been updated to pass one).
        returns:       (B, d_g)
        """
        B = node_features.size(0)
        ctx_list = []
        for i in range(B):
            h = F.gelu(self.gc1(node_features[i], adj[i]))
            h = self.dropout(h)
            h = F.gelu(self.gc2(h, adj[i]))
            h = self.dropout(h)                              # (N, d_g)
            scores  = self.attn_score(h).squeeze(-1)          # (N,)
            if mask is not None:
                scores = scores.masked_fill(mask[i] < 0.5, float('-inf'))
            weights = F.softmax(scores, dim=0)                # (N,)
            # All-masked graphs (shouldn't happen -- every real graph has at
            # least the lanelet the ego is on -- but guard the same way
            # ObjectSetEncoder does for all-padding timesteps) give softmax
            # over all -inf -> NaN; zero those out rather than propagating NaN.
            weights = torch.nan_to_num(weights, nan=0.0)
            ctx_list.append((h * weights.unsqueeze(-1)).sum(dim=0))  # weighted sum, (d_g,)
        return torch.stack(ctx_list)          # (B, d_g)


# ── Object set encoder ──────────────────────────────────────────────────────

class ObjectSetEncoder(nn.Module):
    """
    Permutation-invariant, per-timestep encoder over up to K tracked objects
    (added 2026-08-02, replacing the old object_distance/closest_object_velocity
    scalars). Those scalars collapsed every tracked object to a single
    "nearest object" heuristic BEFORE the model ever saw the data — exactly
    the entity-collapse pattern docs/stgat_pipeline_plan.md's Stage 0 warns
    against (already found and fixed once for traffic lights, §1.11). Here,
    each object keeps its own feature vector; a masked ATTENTION-weighted
    pool over a per-object embedding (DeepSets-style) lets the pooling
    itself be learned rather than hand-picked, and the mask means padding
    slots (fewer than K objects tracked) contribute nothing.

    Was a masked mean-pool (fixed 2026-08-05 — see
    docs/research_notes/ablation_study_2026.md §3, same underlying pattern
    as GraphEncoder's node-pooling fix): a stationary parked car far from
    the route used to count exactly the same as a pedestrian about to
    cross. A learned per-object attention score (masked to -inf on padding
    slots before softmax, so padding gets ~0 weight rather than diluting
    the average) lets the model weight objects by learned relevance.

    Objects are inherently per-timestep (they move within a window,
    independent of whatever cadence the map-node graph uses — see
    docs/theoretical_framework.md §4's discussion of graph cadence), so this
    runs once per timestep, not once per window like GraphEncoder.
    """

    def __init__(self, obj_feat_dim: int, d_obj: int = 32, dropout: float = 0.1):
        super().__init__()
        self.embed = nn.Sequential(
            nn.Linear(obj_feat_dim, d_obj),
            nn.GELU(),
            nn.LayerNorm(d_obj),
            nn.Dropout(dropout),
        )
        self.attn_score = nn.Linear(d_obj, 1)

    def forward(self, objects: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        """
        objects: (B, T, K, F)   mask: (B, T, K) — 1.0 for real objects, 0.0 padding.
        returns: (B, T, d_obj) — attention-weighted object context per timestep.
        """
        B, T, K, Fdim = objects.shape
        h = self.embed(objects.view(B * T * K, Fdim)).view(B, T, K, -1)   # (B, T, K, d_obj)
        scores  = self.attn_score(h).squeeze(-1)                          # (B, T, K)
        scores  = scores.masked_fill(mask < 0.5, float('-inf'))
        weights = F.softmax(scores, dim=2)                                # (B, T, K)
        # All-padding timesteps (no tracked objects at all) give softmax
        # over all -inf -> NaN; zero those out rather than propagating NaN
        # (same "no objects -> zero context" behavior the old mean-pool's
        # counts.clamp(min=1.0) gave for that case).
        weights = torch.nan_to_num(weights, nan=0.0)
        return (h * weights.unsqueeze(-1)).sum(dim=2)                     # (B, T, d_obj)


# ── Main model ─────────────────────────────────────────────────────────────

class STGAT(nn.Module):
    """
    Spatial-Temporal Graph Attention Network for probabilistic trajectory prediction.

    Input dict keys expected in `x`:
        position (B, T, 2), velocity (B, T, 2), steering (B, T, 1),
        acceleration (B, T, 1), traffic_light_color (B, T, 1),
        traffic_light_confidence (B, T, 1), traffic_light_discrepancy (B, T, 1),
        has_adjacent_lane (B, T, 1), uncertainty (B, T, 2),
        objects_set (B, T, K, OBJECT_FEATURE_DIM), objects_mask (B, T, K)

    Output dict:
        position_mean/var  (B, T_out, 2)
        velocity_mean/var  (B, T_out, 2)
        steering_mean/var  (B, T_out)
        acceleration_mean/var (B, T_out)
        traffic_light_color_mean/var (B, T_out) — Gaussian. Perception's
            reported color, most-restrictive element (redesigned 2026-08-05,
            replacing the old collapsed traffic_light_state head — see
            config.py's FEATURE_SIZES doc / docs/research_notes/
            ablation_study_2026.md). A Gaussian head, not a deterministic
            sigmoid, so the predicted variance can itself widen under a
            camera/TL fault.
        traffic_light_confidence_mean/var (B, T_out) — Gaussian. That SAME
            element's confidence, as its own head — split out from color so a
            pure confidence-degradation fault (tl_confidence) and a
            color-changing fault produce distinguishable signal, which the
            old multiplied-scalar traffic_light_state could not.
        traffic_light_discrepancy (B, T_out) — sigmoid-bounded (added
            2026-08-01): binary map-vs-perception mismatch flag, BCE loss.
            traffic_light_detected (the old "map expects a TL here" head) was
            retired entirely 2026-08-05: it was a deterministic geometric fact
            manufactured from ego position + map, redundant with the graph
            itself now that real traffic-light nodes exist there (see
            GraphBuilder.py's _add_traffic_light_nodes) — no output head
            needed for something the GCN can see structurally.

    No output head for objects_set/objects_mask (added 2026-08-02) — input-
    only context via ObjectSetEncoder. Objects don't have a hard map-grounded
    prior the way traffic lights do (docs/theoretical_framework.md §3.1), so
    there's no negative-evidence argument for scoring a prediction against
    them, only for conditioning on them. Also no head for object_distance/
    closest_object_velocity — removed 2026-08-02, replaced by the object set.
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
        d_obj     = config.get('d_object', 32)

        # 12 flat scalar/vector features (position/velocity/steering/accel/
        # TL channels/has_adjacent_lane/uncertainty) + d_obj from the object
        # set encoder (added 2026-08-02, replacing the old object_distance/
        # closest_object_velocity scalars — see ObjectSetEncoder above).
        F_total = sum(config['feature_sizes'].values()) + d_obj

        self.feature_keys = list(config['feature_sizes'].keys())

        # ── Object set branch ────────────────────────────────────────────────
        self.object_encoder = ObjectSetEncoder(
            config['object_feature_dim'], d_obj=d_obj, dropout=dropout,
        )

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
        # traffic_light_color/confidence (redesigned 2026-08-05, replacing
        # the single collapsed head_tl_state — see config.py's FEATURE_SIZES
        # doc). head_traffic (the old traffic_light_detected head) removed
        # entirely at the same time — see the class docstring above.
        self.head_tl_color       = nn.Linear(d_h, 2 * self.T_out)   # mean(1) + var(1)
        self.head_tl_confidence  = nn.Linear(d_h, 2 * self.T_out)   # mean(1) + var(1)
        self.head_tl_discrepancy = nn.Linear(d_h, self.T_out)

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
        node_mask     = graph.get('node_mask')   # (B, N) or None (older cached data)
        B = node_features.size(0)

        # ── 1. Ensure all features are (B, T, dim) ──────────────────────────
        parts = []
        for key in self.feature_keys:
            t = x[key]
            if t.dim() == 2:
                t = t.unsqueeze(-1)
            parts.append(t)

        # Object set context — per-timestep (see ObjectSetEncoder docstring
        # on why this isn't pooled once per window like graph_ctx below).
        obj_ctx = self.object_encoder(x['objects_set'], x['objects_mask'])   # (B, T, d_obj)
        parts.append(obj_ctx)

        x_cat = torch.cat(parts, dim=-1)   # (B, T, F_total)

        # ── 2. Input normalisation ───────────────────────────────────────────
        T = x_cat.size(1)
        x_flat   = x_cat.view(B * T, -1)
        x_flat   = self.input_bn(x_flat)
        x_normed = x_flat.view(B, T, -1)

        # ── 3. Project vehicle features ──────────────────────────────────────
        seq = self.input_proj(x_normed)    # (B, T, d_m)

        # ── 4. Graph context — additive fusion over the time axis ────────────
        graph_ctx  = self.graph_encoder(node_features, adj_matrix, node_mask)  # (B, d_g)
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
        tl_color = self.head_tl_color(h_last).view(B, T_o, 2)
        tl_conf  = self.head_tl_confidence(h_last).view(B, T_o, 2)
        tl_disc  = self.head_tl_discrepancy(h_last).view(B, T_o)

        return {
            'position_mean':    pos[..., :2],
            'position_var':     F.softplus(pos[..., 2:]) + VAR_FLOOR,
            'velocity_mean':    vel[..., :2],
            'velocity_var':     F.softplus(vel[..., 2:]) + VAR_FLOOR,
            'steering_mean':    steer[..., 0],
            'steering_var':     F.softplus(steer[..., 1]) + VAR_FLOOR,
            'acceleration_mean': accel[..., 0],
            'acceleration_var':  F.softplus(accel[..., 1]) + VAR_FLOOR,
            'traffic_light_color_mean':      tl_color[..., 0],
            'traffic_light_color_var':       F.softplus(tl_color[..., 1]) + VAR_FLOOR,
            'traffic_light_confidence_mean': tl_conf[..., 0],
            'traffic_light_confidence_var':  F.softplus(tl_conf[..., 1]) + VAR_FLOOR,
            'traffic_light_discrepancy': torch.sigmoid(tl_disc),
            # Raw pre-sigmoid logit, also returned (added 2026-08-05) so
            # temperature scaling (Guo et al. 2017 — calibrated_prob =
            # sigmoid(logit / T)) can be fit post-hoc without needing to
            # invert the sigmoid on the probability above. No effect on
            # training/loss, which still uses the probability key.
            'traffic_light_discrepancy_logit': tl_disc,
        }

    def count_parameters(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
