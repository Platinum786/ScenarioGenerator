"""planner/trajectory_builder.py — Follow waypoints from entry to exit through junction."""

import carla

MAX_STEPS = 300
STEP_DIST = 0.5   # metres between waypoint steps


class TrajectoryBuilder:
    def build(self, entry_wp: carla.Waypoint, exit_wp: carla.Waypoint) -> list:
        """
        Walk from entry_wp toward exit_wp using next() waypoints.
        Returns a list of carla.Location objects representing the trajectory.
        """
        trajectory = []
        current = entry_wp
        exit_loc = exit_wp.transform.location

        for _ in range(MAX_STEPS):
            loc = current.transform.location
            trajectory.append(loc)

            # Check if we've reached the exit waypoint
            if loc.distance(exit_loc) < 1.5:
                trajectory.append(exit_loc)
                break

            nexts = current.next(STEP_DIST)
            if not nexts:
                break
            # Choose the next waypoint closest to exit
            current = min(nexts, key=lambda wp: wp.transform.location.distance(exit_loc))

        return trajectory if len(trajectory) >= 2 else []
