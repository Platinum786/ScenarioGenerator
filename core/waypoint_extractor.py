"""core/waypoint_extractor.py — Extract entry-exit waypoint pairs from a junction."""

import carla


class WaypointExtractor:
    def __init__(self, junction: carla.Junction):
        self.junction = junction

    def get_entry_exit_pairs(self):
        """
        Returns a list of (entry_waypoint, exit_waypoint) tuples for all
        driving lanes that pass through the junction.
        """
        pairs = self.junction.get_waypoints(carla.LaneType.Driving)
        # Each element is already a tuple (entry_wp, exit_wp)
        return [(entry, exit_) for entry, exit_ in pairs]
