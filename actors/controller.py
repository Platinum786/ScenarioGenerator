"""actors/controller.py — Constant-velocity controller helpers."""

import carla


class VehicleController:
    """
    Thin wrapper around CARLA's enable_constant_velocity.
    Vehicles move at fixed speed; no autopilot required.
    """

    def __init__(self, actor: carla.Actor, velocity: carla.Vector3D):
        self.actor    = actor
        self.velocity = velocity
        self._started = False

    def start(self):
        self.actor.enable_constant_velocity(self.velocity)
        self._started = True

    def stop(self):
        self.actor.enable_constant_velocity(carla.Vector3D(0, 0, 0))
        self._started = False

    @property
    def started(self):
        return self._started
