"""
Minimal map loader, adapted from
Graph-Scene-Representation-and-Prediction/State_Estimator/MapProcessor.py.

Trimmed to just lanelet map loading: takes map_file directly instead of a
global Data_Curator.config instance (route comes from
sequence_builder.extract_route_from_bag() in this pipeline, and the debug
dump method was unused here).
"""

import lanelet2


class MapProcessor:
    def __init__(self, map_file):
        self.map_data = self.load_lanelet_map(map_file)

    def load_lanelet_map(self, file):
        projector = lanelet2.projection.LocalCartesianProjector(lanelet2.io.Origin(35.67, 139.65, 0))
        map_data, load_errors = lanelet2.io.loadRobust(file, projector)
        return map_data
