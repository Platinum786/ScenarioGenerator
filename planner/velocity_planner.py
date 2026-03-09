"""planner/velocity_planner.py — Compute constant speed so vehicle arrives at event_time."""

import math
import carla


class VelocityPlanner:
    def __init__(self, event_time: float):
        """event_time: seconds from scenario start until vehicles should collide."""
        self.event_time = event_time

    def compute(self, distance: float) -> float:
        """
        speed (m/s) = distance / event_time
        Clamps to a minimum of 1 m/s and maximum of 20 m/s.
        """
        speed = distance / max(self.event_time, 0.1)
        return max(1.0, min(speed, 20.0))

    def velocity_vector(self, transform: carla.Transform, speed: float) -> carla.Vector3D:
        """
        Convert yaw from transform into a (vx, vy, 0) velocity vector.
        Uses the forward direction of the spawn transform (which already points at collision).
        """
        yaw_rad = math.radians(transform.rotation.yaw)
        return carla.Vector3D(
            x=speed * math.cos(yaw_rad),
            y=speed * math.sin(yaw_rad),
            z=0.0
        )
