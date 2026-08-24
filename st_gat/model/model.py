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
  - softplus(scale) + SCALE_FLOOR in loss prevents NLL from collapsing to -inf

Total parameters: ~1.2M  (vs ~9M original)

Distribution family and decoder redesigned 2026-08-06 (see
docs/research_notes/trust_and_signal_behavior_2026-08-06.md for the
findings that drove this, and CLAUDE.md's "Direction reframe" for the
priority ordering that put this ahead of resuming SPRT/detection work):

1. Gaussian -> Student-t heads. Coverage-curve/z-score diagnostics on the
   Gaussian-headed model showed all 6 continuous features decisively fail
   an Anderson-Darling normality test (statistic 94-742 against a 5%
   critical value of 0.787) with large positive excess kurtosis (4.8-8.0
   for position/velocity/steering/acceleration; 44-116 for the two TL
   heads) — the real residual distribution is leptokurtic, not Gaussian,
   even though the aggregate std(z) looked fine. Matching only the first
   two moments (mean, variance) under a forced-Gaussian NLL cannot fix a
   shape mismatch; it can only ever get the variance right on average. A
   Student-t predictive distribution has a free degrees-of-freedom (dof)
   parameter controlling tail weight (dof->inf recovers Gaussian; low dof,
   as low as the >2 floor below, gives arbitrarily heavy tails) — the
   model now predicts dof per feature per horizon step alongside mean and
   scale, so it can represent the leptokurtosis directly instead of being
   structurally forced into a bell curve regardless of what the data looks
   like.
2. Flat per-head Linear(d_h, dim*T_out) -> per-horizon-step conditioned
   decoder. The old heads had NO architectural mechanism for horizon step
   t's prediction to differ systematically from step t+1's — each was
   just a disjoint slice of one flat weight matrix, so "does predicted
   uncertainty widen with horizon" had to be learned as an incidental
   pattern in that matrix rather than being structurally supported.
   Measured effect: only `position` (which has an unusually strong,
   ubiquitous training signal for this — position error compounds through
   literal kinematic integration in every window) learned real horizon
   widening; velocity/steering/acceleration/both TL heads' predicted std
   stayed nearly flat across the 3s horizon while actual RMSE grew 2-4x.
   Every head is now a small MLP conditioned on [h_last ; a learned
   per-step horizon embedding] instead of one flat projection — see
   `_StepConditionedHead` below.
3. Per-head horizon embeddings (2026-08-19). Point 2's fix gave every head
   decoder capacity for horizon-dependent behavior, but the horizon
   embedding itself was still ONE table shared across all 7 heads —
   horizon widening still only worked for `position` afterward. A
   gradient-probe diagnostic against the trained checkpoint found why:
   that shared table's gradient was 40-44% owned by position alone (vs.
   6-7% each for steering/acceleration/traffic_light_confidence — exactly
   the three worst-widening heads), and position's gradient direction on
   it had negative cosine similarity (-0.32 to -0.68) with 4 of the other
   5 heads' desired directions — a genuine resource conflict, not just an
   imbalance. Separately: position's target has a real statistical
   advantage the others don't — its marginal (unconditional) std grows
   29x from t=0 to t=29 (kinematic integration), while every other
   feature's marginal std is flat (ratio 0.99-1.01) across the full
   horizon, so position's widening is partly a population-level shortcut
   unavailable to the rest. Each head now gets its own horizon embedding
   table (still tiny — ~3,360 floats total across all 7) instead of one
   shared table, removing the contention mechanism directly.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


# Scale floor applied when computing NLL loss (not in model — see loss.py).
# Analogous to the old VAR_FLOOR (1e-8) but in scale (sigma), not variance,
# units now that heads predict scale directly: sqrt(1e-8) = 1e-4.
SCALE_FLOOR = 1e-4
# Degrees-of-freedom floor for Student-t heads — keeps variance finite
# (requires dof > 2) with a small safety margin against the dof/(dof-2)
# blowup right at the boundary. Deliberately close to 2, not e.g. 5+: the
# leptokurtosis finding above showed some features (traffic_light_color/
# confidence) may genuinely want very heavy tails, and constraining dof
# away from that would just reintroduce a softer version of the same
# forced-Gaussian-ish shape problem this redesign exists to fix.
DOF_FLOOR = 2.1
# Dim of the learned per-horizon-step positional embedding fed into every
# output head (see _StepConditionedHead) — small on purpose, it only needs
# to encode "which of the 30 future steps is this," not carry any of the
# actual predictive signal (that's still h_last's job).
HORIZON_EMBED_DIM = 16


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


# ── Step-conditioned output head ─────────────────────────────────────────────

class _StepConditionedHead(nn.Module):
    """Per-horizon-step decoder head (added 2026-08-06 — see module
    docstring point 2). Replaces a single nn.Linear(d_h, out_dim*T_out)
    with a small shared MLP applied independently at each of the T_out
    future steps, taking [h_last ; horizon_embedding[t]] as input instead
    of h_last alone. This is what gives the model actual architectural
    capacity for horizon-dependent behavior (e.g. widening variance)
    instead of requiring it be learned incidentally inside one flat
    weight matrix's otherwise-unrelated rows.
    """

    def __init__(self, d_h: int, d_pos: int, out_dim: int, hidden: int = 64, dropout: float = 0.1):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(d_h + d_pos, hidden),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden, out_dim),
        )

    def forward(self, h_expanded: torch.Tensor) -> torch.Tensor:
        """h_expanded: (B, T_out, d_h + d_pos) -> (B, T_out, out_dim)."""
        return self.net(h_expanded)


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

    Output dict (Student-t heads, redesigned 2026-08-06 — see module
    docstring point 1 for why Gaussian mean/var was replaced):
        position_mean/scale/dof  (B, T_out, 2) / (B, T_out, 2) / (B, T_out)
        velocity_mean/scale/dof  (B, T_out, 2) / (B, T_out, 2) / (B, T_out)
        steering_mean/scale/dof  (B, T_out)
        acceleration_mean/scale/dof (B, T_out)
        traffic_light_color_mean/scale/dof (B, T_out) — Student-t.
            Perception's reported color, most-restrictive element
            (redesigned 2026-08-05, replacing the old collapsed
            traffic_light_state head — see config.py's FEATURE_SIZES doc /
            docs/research_notes/ablation_study_2026.md). Not a deterministic
            sigmoid, so the predicted scale can itself widen under a
            camera/TL fault; the dof term lets this specific head (measured
            excess kurtosis 44-116 under the old Gaussian assumption — by
            far the most leptokurtic of the 6) predict very heavy tails
            instead of being forced into a bell curve it demonstrably isn't.
        traffic_light_confidence_mean/scale/dof (B, T_out) — Student-t. That
            SAME element's confidence, as its own head — split out from
            color so a pure confidence-degradation fault (tl_confidence)
            and a color-changing fault produce distinguishable signal,
            which the old multiplied-scalar traffic_light_state could not.
        Each feature's dof is shared across its own dims (e.g. position's x
        and y share one dof value per timestep) but predicted per horizon
        step, not a single global constant — see _StepConditionedHead.
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

    # Shared feature-extraction trunk -- everything that produces h_last,
    # i.e. everything upstream of the per-head decoders. See freeze_trunk()
    # below (2026-08-20).
    _TRUNK_MODULES = ('object_encoder', 'graph_encoder', 'graph_proj',
                       'input_bn', 'input_proj', 'attn_block', 'lstm')

    def __init__(self, config: dict):
        super().__init__()

        # distribution: 'student_t' (default, unchanged from 2026-08-06) or
        # 'gaussian' (added 2026-08-07 for STGATEnsemble -- see ensemble.py
        # and docs/research_notes/model_redesign_literature_2026-08-07.md).
        # Gaussian mode drops the dof parameter entirely (mean+var per head,
        # the pre-2026-08-06 schema) -- deliberate reversion for ensemble
        # members specifically: Lakshminarayanan et al.'s aleatoric/epistemic
        # combination formula (law of total variance) is exact for a mixture
        # of Gaussians, not as clean for a mixture of Student-t's, and
        # ensemble disagreement is expected to absorb some of what the
        # single-network Student-t redesign needed heavy tails to represent
        # (see the design doc's falsifiable hypothesis on this). The
        # Student-t path (still the default) is completely unaffected by
        # this addition -- every existing script keeps working unchanged.
        self.distribution = config.get('distribution', 'student_t')
        assert self.distribution in ('student_t', 'gaussian'), self.distribution

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

        # ── Output heads (2026-08-06: per-horizon-step conditioned, Student-t
        # -- see module docstring points 1/2 and _StepConditionedHead) ──────
        # Each continuous head outputs mean(dims) + scale(dims) + dof(1,
        # shared across dims) per step; traffic_light_discrepancy is
        # unchanged (Bernoulli/sigmoid, no distribution-family question).
        d_pos = HORIZON_EMBED_DIM
        self.register_buffer('_horizon_idx', torch.arange(self.T_out), persistent=False)

        # Per-head horizon embeddings (fixed 2026-08-19 — see module
        # docstring point 3): this used to be ONE nn.Embedding shared across
        # all 7 heads. A gradient-probe diagnostic against the trained
        # checkpoint found that shared table's gradient was 40-44% owned by
        # position alone (vs. 6-7% each for steering/acceleration/
        # traffic_light_confidence — exactly the three worst-widening
        # heads), AND position's gradient direction on that table had
        # negative cosine similarity (-0.32 to -0.68) with 4 of the other 5
        # heads' desired directions — not just outweighed, actively fought.
        # Separately, position's own target has a genuinely different
        # statistical structure: its marginal (unconditional) std grows 29x
        # from t=0 to t=29 (kinematic integration), while every other
        # feature's marginal std is flat (ratio 0.99-1.01) across the full
        # horizon — so position gets a population-level widening shortcut
        # none of the other heads have, making it even more likely to
        # dominate a shared resource. One embedding table PER head (still
        # tiny — 30*16*7 ~= 3,360 floats total) removes the contention
        # mechanism directly, at negligible parameter/compute cost.
        _head_keys = ('position', 'velocity', 'steering', 'acceleration',
                      'traffic_light_color', 'traffic_light_confidence',
                      'traffic_light_discrepancy')
        self.horizon_embed = nn.ModuleDict({
            k: nn.Embedding(self.T_out, d_pos) for k in _head_keys
        })

        # Per-dim output width: student_t = mean+scale+dof-shared (2*dims+1);
        # gaussian = mean+var (2*dims), no dof term at all.
        _w2 = (lambda dims: 2 * dims + 1) if self.distribution == 'student_t' else (lambda dims: 2 * dims)
        self.head_position    = _StepConditionedHead(d_h, d_pos, _w2(2))
        self.head_velocity    = _StepConditionedHead(d_h, d_pos, _w2(2))
        self.head_steering    = _StepConditionedHead(d_h, d_pos, _w2(1))
        self.head_accel       = _StepConditionedHead(d_h, d_pos, _w2(1))
        # traffic_light_color/confidence (redesigned 2026-08-05, replacing
        # the single collapsed head_tl_state — see config.py's FEATURE_SIZES
        # doc). head_traffic (the old traffic_light_detected head) removed
        # entirely at the same time — see the class docstring above.
        self.head_tl_color       = _StepConditionedHead(d_h, d_pos, _w2(1))
        self.head_tl_confidence  = _StepConditionedHead(d_h, d_pos, _w2(1))
        self.head_tl_discrepancy = _StepConditionedHead(d_h, d_pos, 1)

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

    def encode_scene(self, x: dict, graph: dict) -> torch.Tensor:
        """Steps 1-6 of forward(): everything upstream of the per-head
        decoders, producing h_last -- the single vector every head is
        conditioned on. Factored out (2026-08-24) so callers that need the
        model's own learned "what kind of situation is this" representation
        (e.g. scene-embedding-conditioned conformal quantiles, see
        experiments/scripts/conformal_scene_conditioning.py) can get it
        without duplicating trunk logic that forward() also needs to stay
        in sync with. forward() calls this directly -- there is exactly one
        implementation of the trunk, not two copies that could drift."""
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
        return self.lstm_drop(lstm_out[:, -1])   # (B, d_h) — last hidden state

    def forward(self, x: dict, graph: dict) -> dict:
        B = graph['node_features'].size(0)
        h_last = self.encode_scene(x, graph)

        # ── 7. Output heads — per-horizon-step conditioned (2026-08-06) ──────
        T_o = self.T_out
        h_expanded = h_last.unsqueeze(1).expand(-1, T_o, -1)             # (B, T_o, d_h)

        def _h_cond(key: str) -> torch.Tensor:
            """[h_last ; that head's OWN horizon embedding[t]] — see
            __init__'s per-head horizon_embed note for why this is no
            longer one embedding shared across heads."""
            pos_embed = self.horizon_embed[key](self._horizon_idx)         # (T_o, d_pos)
            pos_embed = pos_embed.unsqueeze(0).expand(B, -1, -1)           # (B, T_o, d_pos)
            return torch.cat([h_expanded, pos_embed], dim=-1)              # (B, T_o, d_h + d_pos)

        pos   = self.head_position(_h_cond('position'))       # (B, T_o, 5): mean(2)+scale(2)+dof(1)
        vel   = self.head_velocity(_h_cond('velocity'))        # (B, T_o, 5)
        steer = self.head_steering(_h_cond('steering'))        # (B, T_o, 3): mean(1)+scale(1)+dof(1)
        accel = self.head_accel(_h_cond('acceleration'))       # (B, T_o, 3)
        tl_color = self.head_tl_color(_h_cond('traffic_light_color'))      # (B, T_o, 3)
        tl_conf  = self.head_tl_confidence(_h_cond('traffic_light_confidence'))  # (B, T_o, 3)
        tl_disc  = self.head_tl_discrepancy(_h_cond('traffic_light_discrepancy')).squeeze(-1)   # (B, T_o)

        def _dof(raw_dof: torch.Tensor) -> torch.Tensor:
            # dof > DOF_FLOOR always (finite variance) — see module docstring.
            return DOF_FLOOR + F.softplus(raw_dof)

        def _split(raw: torch.Tensor, key: str, dims: int) -> dict:
            """Slice one head's raw (B, T_o, out_dim) output into named,
            correctly-shaped outputs for the active distribution -- see
            __init__'s _w2 for the matching out_dim convention. Scalar
            features (dims=1) get squeezed to (B, T_o), matching every
            existing consumer's convention (same as the old hand-written
            per-feature slicing this replaces)."""
            mean = raw[..., :dims]
            if self.distribution == 'gaussian':
                var = F.softplus(raw[..., dims:2 * dims]) + SCALE_FLOOR
                if dims == 1:
                    mean, var = mean.squeeze(-1), var.squeeze(-1)
                return {f'{key}_mean': mean, f'{key}_var': var}
            else:
                scale = F.softplus(raw[..., dims:2 * dims]) + SCALE_FLOOR
                dof   = _dof(raw[..., 2 * dims])   # already (B, T_o), one dof per step shared across dims
                if dims == 1:
                    mean, scale = mean.squeeze(-1), scale.squeeze(-1)
                return {f'{key}_mean': mean, f'{key}_scale': scale, f'{key}_dof': dof}

        out = {}
        out.update(_split(pos,      'position',                  2))
        out.update(_split(vel,      'velocity',                  2))
        out.update(_split(steer,    'steering',                  1))
        out.update(_split(accel,    'acceleration',               1))
        out.update(_split(tl_color, 'traffic_light_color',       1))
        out.update(_split(tl_conf,  'traffic_light_confidence',  1))
        out['traffic_light_discrepancy'] = torch.sigmoid(tl_disc)
        # Raw pre-sigmoid logit, also returned (added 2026-08-05) so
        # temperature scaling (Guo et al. 2017 — calibrated_prob =
        # sigmoid(logit / T)) can be fit post-hoc without needing to invert
        # the sigmoid on the probability above. No effect on training/loss,
        # which still uses the probability key.
        out['traffic_light_discrepancy_logit'] = tl_disc
        return out

    def count_parameters(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)

    def freeze_trunk(self, freeze: bool = True):
        """Freeze (or unfreeze) every shared trunk module (_TRUNK_MODULES),
        leaving each head's own MLP + horizon embedding trainable (added
        2026-08-20 -- see docs/research_notes/, the 2026-08-19/20
        checkpoint-selection-divergence investigation).

        Motivation: a data check on a completed Phase-2 run found
        position/velocity point accuracy improving monotonically the ENTIRE
        run while validation total_nll got monotonically WORSE, in lockstep,
        for all 6 heads simultaneously -- not the classic Wong-Toi-style
        per-head variance collapse (VAR_REG_WEIGHT, which only fights scale
        shrinking below the empirical residual, was confirmed inactive/flat
        for most heads and 5x-ing it didn't change the trajectory). The
        shared-lockstep-degradation shape instead points at the shared
        trunk (graph/temporal encoder -> h_last) continuing to adapt under
        Phase 2's optimizer, driven by whichever head's gradient is
        largest (point-accuracy-adjacent signal, per this session's own
        gradient-dominance finding for horizon_embed) -- improving the mean
        at the expense of what the now-per-head scale/dof heads need from
        that shared representation to generalize to held-out data. Freezing
        the trunk for Phase 2 tests this directly: the only remaining
        trainable parameters are each head's own small MLP (already fully
        separate per feature, no shared weights) plus its own horizon
        embedding (already split per-head, see model.py's earlier
        2026-08-19 fix) -- so this is equivalent to fine-tuning each
        feature's own scale/dof independently against a FIXED shared
        representation, with zero remaining cross-feature parameter
        contention."""
        for name in self._TRUNK_MODULES:
            for p in getattr(self, name).parameters():
                p.requires_grad = not freeze
