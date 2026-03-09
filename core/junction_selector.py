"""core/junction_selector.py — Scan the map and select a junction of the requested type."""

import random
import carla


class JunctionSelector:
    def __init__(self, world: carla.World, junction_type: str):
        """
        junction_type: "3_way" or "4_way"
        """
        self.world = world
        self.junction_type = junction_type
        self._carla_map = world.get_map()

    # ------------------------------------------------------------------
    def _count_road_connections(self, junction: carla.Junction) -> int:
        """
        Estimate the number of road arms connecting to this junction by
        counting distinct road IDs among all entry waypoints.
        """
        pairs = junction.get_waypoints(carla.LaneType.Driving)
        road_ids = set()
        for entry_wp, _ in pairs:
            road_ids.add(entry_wp.road_id)
        return len(road_ids)

    # ------------------------------------------------------------------
    def select(self) -> carla.Junction:
        """Return a randomly selected junction matching the requested type."""
        target_arms = 3 if self.junction_type == "3_way" else 4

        all_waypoints = self._carla_map.generate_waypoints(2.0)
        junction_ids_seen = set()
        candidates = []

        for wp in all_waypoints:
            if wp.is_junction:
                j = wp.get_junction()
                if j.id in junction_ids_seen:
                    continue
                junction_ids_seen.add(j.id)
                arms = self._count_road_connections(j)
                if arms == target_arms:
                    candidates.append(j)

        if not candidates:
            # Fallback: accept any junction
            print(f"[Warning] No strict {self.junction_type} junction found — using any junction.")
            for wp in all_waypoints:
                if wp.is_junction:
                    j = wp.get_junction()
                    if j.id not in junction_ids_seen:
                        candidates.append(j)
                        junction_ids_seen.add(j.id)

        if not candidates:
            raise RuntimeError("No junctions found in the map.")

        chosen = random.choice(candidates)
        print(f"[JunctionSelector] Found {len(candidates)} candidates, chose id={chosen.id}")
        return chosen

    # ------------------------------------------------------------------
    @staticmethod
    def center(junction: carla.Junction) -> carla.Location:
        """Approximate junction centre from its bounding box."""
        bb = junction.bounding_box
        return bb.location
