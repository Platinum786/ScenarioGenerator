"""planner/lane_intersection_detector.py — Find where two trajectories cross."""

import carla
import itertools


class LaneIntersectionDetector:
    def __init__(self, threshold: float = 2.5):
        """threshold: max distance (m) between two trajectory points to count as intersection."""
        self.threshold = threshold

    def find_intersection(self, trajectories: list):
        """
        Compare every pair of trajectories and return the first collision point.

        Returns:
            (collision_point: carla.Location, traj_pair: (list, list))
            or (None, None) if no intersection found.
        """
        for traj_a, traj_b in itertools.combinations(trajectories, 2):
            # Skip trajectories that share the same entry road (same direction)
            if traj_a[0].distance(traj_b[0]) < 2.0:
                continue

            for pt_a in traj_a:
                for pt_b in traj_b:
                    dist = pt_a.distance(pt_b)
                    if dist < self.threshold:
                        # Collision point = midpoint
                        mid = carla.Location(
                            x=(pt_a.x + pt_b.x) / 2,
                            y=(pt_a.y + pt_b.y) / 2,
                            z=(pt_a.z + pt_b.z) / 2,
                        )
                        return mid, (traj_a, traj_b)

        return None, None
