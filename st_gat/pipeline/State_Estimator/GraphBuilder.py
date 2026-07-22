import networkx as nx
from ..Data_Curator.Point import Point


class GraphBuilder:
    def __init__(self, map_data, route, min_dist_between_node, connection_threshold, max_nodes, min_nodes):
        self.map_data = map_data
        self.route = route
        self.min_dist_between_node = min_dist_between_node
        self.connection_threshold = connection_threshold
        self.max_nodes = max_nodes
        self.min_nodes = min_nodes

    def _get_lanelet_mid_point(self, lanelet):
        centerline = lanelet.centerline
        mid_index = len(centerline) // 2
        return Point(centerline[mid_index].x, centerline[mid_index].y)

    def _get_sorted_lanelets(self, center_position):
        lanelets = []
        for ll in self.map_data.laneletLayer:
            if ll.attributes["subtype"] == "road":
                mid_point = self._get_lanelet_mid_point(ll)
                distance = Point.distance(center_position, mid_point)
                lanelets.append((ll.id, ll, distance))

        return sorted(lanelets, key=lambda x: x[2])

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
                    'traffic_light_detection_node': 0,
                    'path_node': 1 if lanelet_id in self.route else 0
                })
                prev_point = current_point
        return nodes

    def clip_graph(self, G, center_position):
        if G.number_of_nodes() <= self.min_nodes:
            return G

        # Calculate distances from center to all nodes
        distances = {}
        for node, data in G.nodes(data=True):
            node_point = Point(data['x'], data['y'])
            distance = Point.distance(center_position, node_point)
            distances[node] = distance

        # Sort nodes by distance from center
        sorted_nodes = sorted(distances.items(), key=lambda x: x[1])

        # Keep only the closest min_nodes
        nodes_to_keep = [node for node, _ in sorted_nodes[:self.min_nodes]]
        nodes_to_remove = set(G.nodes()) - set(nodes_to_keep)

        # Remove furthest nodes
        G.remove_nodes_from(nodes_to_remove)

        return G

    def _connect_closest_components(self, G, components):
        min_distance = float('inf')
        closest_pair = None
        for i, comp1 in enumerate(components):
            for j, comp2 in enumerate(components[i+1:], i+1):
                for node1 in comp1:
                    for node2 in comp2:
                        distance = Point.distance(Point(G.nodes[node1]['x'], G.nodes[node1]['y']),
                                                  Point(G.nodes[node2]['x'], G.nodes[node2]['y']))
                        if distance < min_distance:
                            min_distance = distance
                            closest_pair = (node1, node2)
        if closest_pair:
            G.add_edge(*closest_pair, type="connection_edge")

    def _ensure_graph_connectivity(self, G):
        if not nx.is_connected(G):
            components = list(nx.connected_components(G))
            while len(components) > 1:
                self._connect_closest_components(G, components)
                components = list(nx.connected_components(G))

    def update_traffic_lights(self, G):
        for ls in self.map_data.lineStringLayer:
            if ls.attributes["type"] == "traffic_light":
                mid_point = Point.get_mid_point(Point(ls[0].x, ls[0].y), Point(ls[1].x, ls[1].y))
                for node in G.nodes(data=True):
                    node_point = Point(node[1]['x'], node[1]['y'])
                    distance = Point.distance(mid_point, node_point)
                    if distance <= 10:
                        G.nodes[node[0]]['traffic_light_detection_node'] = 1

    def build_graph(self, center_position):
        G = nx.Graph()
        lanelets = self._get_sorted_lanelets(center_position)

        added_nodes = 0
        for lanelet_id, lanelet, _ in lanelets:
            nodes = self._create_lanelet_nodes(lanelet, lanelet_id)

            for i, node_data in enumerate(nodes):
                if added_nodes >= self.max_nodes:
                    break
                node_id = G.number_of_nodes()
                G.add_node(node_id, **node_data)

                # Add edge to previous node in the same lanelet
                if i > 0:
                    prev_node_id = node_id - 1
                    G.add_edge(prev_node_id, node_id, type="lane_edge", lanelet_id=lanelet_id)

                added_nodes += 1

            if added_nodes >= self.max_nodes:
                break

        self.update_traffic_lights(G)
        self.clip_graph(G, center_position)
        self._ensure_graph_connectivity(G)
        return G

    def create_expanded_graph(self, initial_position, final_position):
        center_x = (initial_position.x + final_position.x) / 2
        center_y = (initial_position.y + final_position.y) / 2
        center_position = Point(center_x, center_y)

        return self.build_graph(center_position)
