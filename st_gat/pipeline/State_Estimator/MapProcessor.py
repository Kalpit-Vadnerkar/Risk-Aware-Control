"""
Minimal map loader — trimmed to just lanelet map loading: takes map_file directly
instead of a global config instance (route comes from
sequence_builder.extract_route_from_bag() in this pipeline).
"""

import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector


class MapProcessor:
    def __init__(self, map_file):
        self.map_data = self.load_lanelet_map(map_file)

    def load_lanelet_map(self, file):
        # Matches Autoware's own map loader (autoware_map_projection_loader):
        # when no map_projector_info.yaml is present, Autoware projects the
        # lanelet2 map with MGRSProjector, deriving the MGRS grid cell from the
        # map's own lat/lon points. A generic LocalCartesianProjector/UtmProjector
        # with a guessed lat/lon origin does NOT line up with the AWSIM/Autoware
        # map frame (verified off by tens of km) — MGRSProjector is required for
        # ego positions from the bag (already in the AWSIM/MGRS frame) to align
        # with this map's coordinates.
        projector = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
        map_data, load_errors = lanelet2.io.loadRobust(file, projector)
        return map_data
