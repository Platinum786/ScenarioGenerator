"""planner/spawn_planner.py — Choose spawn points on trajectories before the collision point."""

import math
import carla


class SpawnPlanner:
    def __init__(self, spawn_distance: float = 50.0):
        """spawn_distance: target distance (m) back from collision point to spawn vehicles."""
        self.spawn_distance = spawn_distance

    # ------------------------------------------------------------------
    def _find_spawn_on_trajectory(
        self, trajectory: list, collision_point: carla.Location
    ):
        """
        Walk along the trajectory and find the point closest to spawn_distance
        metres before the collision point.
        Returns (spawn_location, distance_to_collision, yaw_degrees).
        """
        # Build cumulative distance from start
        cum_dist = [0.0]
        for i in range(1, len(trajectory)):
            cum_dist.append(cum_dist[-1] + trajectory[i - 1].distance(trajectory[i]))

        # Find trajectory index closest to collision point
        col_idx = min(range(len(trajectory)),
                      key=lambda i: trajectory[i].distance(collision_point))

        # Distance from start to collision index along trajectory
        dist_to_col = cum_dist[col_idx]

        # Target cumulative distance = dist_to_col - spawn_distance
        target_cum = max(0.0, dist_to_col - self.spawn_distance)

        spawn_idx = min(
            range(len(cum_dist)),
            key=lambda i: abs(cum_dist[i] - target_cum)
        )
        spawn_loc = trajectory[spawn_idx]

        # Distance along trajectory from spawn to collision
        dist_along = dist_to_col - cum_dist[spawn_idx]
        dist_along = max(dist_along, 1.0)  # safety floor

        # Yaw toward collision point
        dx = collision_point.x - spawn_loc.x
        dy = collision_point.y - spawn_loc.y
        yaw = math.degrees(math.atan2(dy, dx))

        return spawn_loc, dist_along, yaw

    # ------------------------------------------------------------------
    def plan(self, traj_pair: tuple, collision_point: carla.Location, npc_count: int) -> list:
        """
        Returns a list of dicts, one per vehicle:
            {"transform": carla.Transform, "distance": float}
        Index 0 → ego vehicle, 1..N → NPC vehicles.
        """
        traj_a, traj_b = traj_pair
        results = []

        for traj in [traj_a, traj_b][: npc_count + 1]:
            spawn_loc, dist, yaw = self._find_spawn_on_trajectory(traj, collision_point)
            transform = carla.Transform(
                carla.Location(x=spawn_loc.x, y=spawn_loc.y, z=spawn_loc.z + 0.3),
                carla.Rotation(yaw=yaw)
            )
            results.append({"transform": transform, "distance": dist})

        return results
