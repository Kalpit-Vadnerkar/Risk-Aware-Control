import networkx as nx
from ..Data_Curator.Point import Point


def _reg_attr(reg, key, default=None):
    """Same defensive pattern as experiments/lib/plotting.py's ll_attr() —
    not every regulatory element carries every attribute key."""
    try:
        return reg.attributes[key]
    except Exception:
        return default


def regulatory_element_position(reg):
    """Physical (x, y) of a traffic-light regulatory element: the average of
    all its referred linestrings' points. Identical computation to
    experiments/scripts/compute_tl_zones.py's route_tl_zones() (the function
    fault_injector.py/tl_zones.json use to pick which real-world traffic
    light a TL fault targets) — kept in sync deliberately so "the traffic
    light a fault mutates" and "the traffic light node in this graph" refer
    to the same real-world entity, not two independently-derived
    approximations of it."""
    try:
        refers = reg.parameters['refers']
    except Exception:
        return None
    all_pts = [(p.x, p.y) for ls in refers for p in ls]
    if not all_pts:
        return None
    return sum(p[0] for p in all_pts) / len(all_pts), sum(p[1] for p in all_pts) / len(all_pts)


class GraphBuilder:
    def __init__(self, map_data, route, min_dist_between_node, connection_threshold,
                 max_nodes, radius_m, routing_graph=None):
        self.map_data = map_data
        # O(1) membership -- was a plain list, checked via `ll.id in self.route`
        # (O(len(route))) once per candidate lanelet on every graph rebuild.
        self.route = set(route)
        self.min_dist_between_node = min_dist_between_node
        self.connection_threshold = connection_threshold
        self.max_nodes = max_nodes
        self.radius_m = radius_m
        # lanelet2.routing.RoutingGraph -- shared, not rebuilt per graph: it
        # only depends on map_data (never changes across trials), so
        # SequenceBuilder.__init__ builds it once and passes the same object
        # in here and into LaneletAdjacencyChecker. None disables
        # topology-aware connectivity (falls back to the distance-capped
        # nearest-pair repair only).
        self.routing_graph = routing_graph

    def _get_lanelet_mid_point(self, lanelet):
        centerline = lanelet.centerline
        mid_index = len(centerline) // 2
        return Point(centerline[mid_index].x, centerline[mid_index].y)

    def _get_sorted_lanelets(self, center_position):
        # Radius-gated, route-aware fill order (audited/fixed 2026-08-05 --
        # see docs/research_notes/ablation_study_2026.md). Route-first
        # sorting (§2) fixed route lanelets losing the node budget to nearby
        # off-route ones in dense intersections, but without a radius cap it
        # overshot the other way: an on-route lanelet arbitrarily far away
        # (measured up to 656m in one real trial) always wins over ANY
        # off-route lanelet regardless of distance, so the entire node
        # budget could go to one long, mostly-irrelevant route ribbon while
        # genuinely local off-route context (a cross-traffic lanelet at an
        # intersection, an adjacent lane) was excluded entirely. Filtering
        # candidates to radius_m FIRST makes route-preference a tiebreaker
        # among locally-relevant lanelets, not an unbounded override of
        # distance -- see config.py's GRAPH_RADIUS_M for how the radius
        # itself is derived from the prediction horizon.
        lanelets = []
        for ll in self.map_data.laneletLayer:
            if ll.attributes["subtype"] != "road":
                continue
            mid_point = self._get_lanelet_mid_point(ll)
            distance = Point.distance(center_position, mid_point)
            if distance > self.radius_m:
                continue
            on_route = ll.id in self.route
            lanelets.append((ll.id, ll, distance, on_route))

        return sorted(lanelets, key=lambda x: (not x[3], x[2]))

    def _create_lanelet_nodes(self, lanelet, lanelet_id):
        nodes = []
        prev_point = None
        for point in lanelet.centerline:
            current_point = Point(point.x, point.y)
            if prev_point is None or Point.distance(prev_point, current_point) >= self.min_dist_between_node:
                nodes.append({
                    'type': "map_node",
                    'x': current_point.x,
                    'y': current_point.y,
                    # Real traffic-light nodes are added separately as their
                    # own dedicated nodes (see _add_traffic_light_nodes) --
                    # lane nodes themselves never carry this flag anymore.
                    'traffic_light_detection_node': 0,
                    'path_node': 1 if lanelet_id in self.route else 0
                })
                prev_point = current_point
        return nodes

    def _add_traffic_light_nodes(self, G, lanelet_node_range, lanelet_by_id):
        """Add one dedicated graph node per distinct traffic-light
        regulatory element governing a lanelet actually included in this
        graph, positioned at the light's real physical location (see
        regulatory_element_position above) -- replaces the old
        update_traffic_lights(), which instead flagged whichever EXISTING
        lane-centerline node happened to fall within 10m of a TL
        linestring. That made "is there a TL here" a manufactured proxy
        riding on a node whose position was really about lane geometry, not
        the light itself; a real node at the light's own coordinates lets
        the GCN learn the spatial relationship directly instead of through
        that proxy (see config.py's FEATURE_SIZES doc for why
        traffic_light_detected was retired as an output feature at the same
        time this was added). Only considers lanelets already included in
        THIS graph (not the whole route) so a light 600m down the route
        doesn't get a node here any more than its governing lanelet does.
        """
        if G.number_of_nodes() == 0:
            return

        seen_reg_ids = set()
        for lanelet_id in lanelet_node_range:
            ll = lanelet_by_id[lanelet_id]
            for reg in ll.regulatoryElements:
                if _reg_attr(reg, "subtype") != "traffic_light" or reg.id in seen_reg_ids:
                    continue
                pos = regulatory_element_position(reg)
                if pos is None:
                    continue
                seen_reg_ids.add(reg.id)

                # Nearest EXISTING node (lane node or an earlier TL node) --
                # computed before adding the new node so it can't be its own
                # nearest neighbor.
                nearest_id, _ = min(
                    ((n, (d['x'] - pos[0]) ** 2 + (d['y'] - pos[1]) ** 2)
                     for n, d in G.nodes(data=True)),
                    key=lambda t: t[1],
                )

                tl_node_id = G.number_of_nodes()
                G.add_node(tl_node_id, type="traffic_light_node", x=pos[0], y=pos[1],
                           traffic_light_detection_node=1, path_node=0,
                           traffic_light_group_id=reg.id)
                # A real, known-good connection -- not a repair, so it
                # doesn't count against connection_threshold's budget.
                G.add_edge(nearest_id, tl_node_id, type="tl_edge")

    def _connect_lanelet_topology(self, G, lanelet_node_range, lanelet_by_id):
        """Connect included lanelets to each other using REAL lanelet2
        routing relations (successor / adjacent-left / adjacent-right)
        instead of relying entirely on the reactive nearest-pair repair in
        _ensure_graph_connectivity (added 2026-08-05 -- see
        docs/research_notes/ablation_study_2026.md: the repair step was
        firing on effectively every graph build, 60% of a trial's total
        extraction runtime, precisely because lanelets were never connected
        to each other by anything but coincidence). Only connects pairs
        where BOTH lanelets are actually included in this graph -- the
        radius/node-budget may have excluded one side."""
        if self.routing_graph is None:
            return

        for lanelet_id, (first_id, last_id) in lanelet_node_range.items():
            ll = lanelet_by_id[lanelet_id]

            for nxt in self.routing_graph.following(ll):
                if nxt.id in lanelet_node_range:
                    nxt_first, _nxt_last = lanelet_node_range[nxt.id]
                    G.add_edge(last_id, nxt_first, type="lane_topology_edge", kind="successor")

            for prev in self.routing_graph.previous(ll):
                if prev.id in lanelet_node_range:
                    _prev_first, prev_last = lanelet_node_range[prev.id]
                    G.add_edge(prev_last, first_id, type="lane_topology_edge", kind="successor")

            left = self.routing_graph.adjacentLeft(ll)
            if left is not None and left.id in lanelet_node_range:
                l_first, l_last = lanelet_node_range[left.id]
                G.add_edge(first_id, l_first, type="lane_topology_edge", kind="adjacent")
                G.add_edge(last_id, l_last, type="lane_topology_edge", kind="adjacent")

            right = self.routing_graph.adjacentRight(ll)
            if right is not None and right.id in lanelet_node_range:
                r_first, r_last = lanelet_node_range[right.id]
                G.add_edge(first_id, r_first, type="lane_topology_edge", kind="adjacent")
                G.add_edge(last_id, r_last, type="lane_topology_edge", kind="adjacent")

    def _connect_remaining_fragments(self, G, components):
        """Bridge every fragment left over after topology-aware connectivity
        (_connect_lanelet_topology) into the single largest component, via a
        KD-tree nearest-node query per fragment -- NOT the brute-force
        O(sum_i<j |c_i|*|c_j|) all-pairs-of-all-components search this
        replaced (2026-08-05). That search stayed the dominant cost even
        after adding topology-aware connections (measured: 83% of a
        13.9s-total trial, down from 106s before any of this session's
        fixes, but still the long pole) -- it's the wrong asymptotic
        approach once a component can have 100+ nodes, not something a
        distance cap alone would fix. Expected to fire routinely, not
        rarely: radius-bounded lanelet selection inherently produces small
        dangling fragments whose only path back to the main cluster exits
        the radius window -- that's a structural consequence of ANY bounded
        neighborhood query, not a bug in the topology connections.
        Connecting each fragment independently to its own nearest point in
        the main cluster (rather than iteratively finding the single
        globally-closest cross-component pair, which could chain two small
        fragments to each other before either reaches the real road
        network) is also the more sensible bridge topology, not just
        faster.
        """
        from scipy.spatial import cKDTree
        import numpy as np

        components = sorted(components, key=len, reverse=True)
        positions = {n: (G.nodes[n]['x'], G.nodes[n]['y']) for n in G.nodes()}

        main_nodes = list(components[0])
        main_xy    = np.array([positions[n] for n in main_nodes])

        for comp in components[1:]:
            tree = cKDTree(main_xy)
            comp_nodes = list(comp)
            comp_xy    = np.array([positions[n] for n in comp_nodes])
            dists, idxs = tree.query(comp_xy)
            best = int(np.argmin(dists))
            best_dist  = float(dists[best])
            comp_node  = comp_nodes[best]
            main_node  = main_nodes[idxs[best]]

            # connection_threshold is a genuine cap now (2026-08-05 -- it was
            # accepted as a constructor argument and stored but never
            # actually read anywhere in this class, so a bridge could
            # previously span an arbitrary distance with no visibility at
            # all). Still bridges past the cap -- refusing to would leave
            # the graph disconnected -- but now logs it, since it's worth
            # knowing whether a given bridge is routine (a dangling fragment
            # just outside the radius) or a genuine map/route gap.
            if best_dist > self.connection_threshold:
                print(f"  [GraphBuilder] WARNING: nearest cross-fragment pair is "
                      f"{best_dist:.1f}m apart (> connection_threshold="
                      f"{self.connection_threshold}m) -- bridging anyway to keep "
                      f"the graph connected; check whether this is a genuine "
                      f"map/route disconnect.")
            G.add_edge(comp_node, main_node, type="connection_edge")

            # Fold this fragment into the growing main set so later
            # fragments can bridge to it too, without rebuilding the KD-tree
            # from a fresh nx.connected_components() scan each time.
            main_nodes.extend(comp_nodes)
            main_xy = np.vstack([main_xy, comp_xy])

    def _ensure_graph_connectivity(self, G):
        if not nx.is_connected(G):
            components = list(nx.connected_components(G))
            self._connect_remaining_fragments(G, components)

    def _count_traffic_light_candidates(self, lanelets) -> int:
        """Distinct traffic-light regulatory elements among candidate
        lanelets, counted BEFORE the lane-node fill loop runs -- used to
        reserve node budget for _add_traffic_light_nodes (see build_graph).
        Conservative: counts every candidate lanelet's regulatory elements,
        not just the ones that end up included after the fill loop, so this
        never under-reserves (which would silently drop a real TL node
        again) at the cost of occasionally reserving a slot or two more
        than strictly needed."""
        seen_reg_ids = set()
        for _lid, ll, _dist, _on_route in lanelets:
            for reg in ll.regulatoryElements:
                if _reg_attr(reg, "subtype") == "traffic_light":
                    seen_reg_ids.add(reg.id)
        return len(seen_reg_ids)

    def build_graph(self, center_position):
        G = nx.Graph()
        lanelets = self._get_sorted_lanelets(center_position)
        lanelet_by_id = {lid: ll for lid, ll, _dist, _on_route in lanelets}

        # Reserve budget for real TL nodes BEFORE filling lane nodes (added
        # 2026-08-05 -- found via direct testing: a graph whose lane-node
        # fill loop alone consumed the full max_nodes budget pushed every TL
        # node's id past max_nodes, and dataset.py's fixed-size padding
        # tensor silently drops any node_id >= max_nodes -- so the real TL
        # nodes this redesign added were, in the case actually tested,
        # never reaching the model at all).
        n_reserved_tl = self._count_traffic_light_candidates(lanelets)
        lane_node_budget = max(self.max_nodes - n_reserved_tl, 0)

        added_nodes = 0
        lanelet_node_range = {}   # lanelet_id -> (first_node_id, last_node_id)
        for lanelet_id, lanelet, _distance, _on_route in lanelets:
            nodes = self._create_lanelet_nodes(lanelet, lanelet_id)

            first_id = None
            for i, node_data in enumerate(nodes):
                if added_nodes >= lane_node_budget:
                    break
                node_id = G.number_of_nodes()
                G.add_node(node_id, **node_data)
                if first_id is None:
                    first_id = node_id

                # Add edge to previous node in the same lanelet
                if i > 0:
                    prev_node_id = node_id - 1
                    G.add_edge(prev_node_id, node_id, type="lane_edge", lanelet_id=lanelet_id)

                added_nodes += 1

            if first_id is not None:
                lanelet_node_range[lanelet_id] = (first_id, node_id)

            if added_nodes >= lane_node_budget:
                break

        self._connect_lanelet_topology(G, lanelet_node_range, lanelet_by_id)
        self._add_traffic_light_nodes(G, lanelet_node_range, lanelet_by_id)
        self._ensure_graph_connectivity(G)
        return G

    def create_expanded_graph(self, initial_position, final_position):
        center_x = (initial_position.x + final_position.x) / 2
        center_y = (initial_position.y + final_position.y) / 2
        center_position = Point(center_x, center_y)

        return self.build_graph(center_position)
