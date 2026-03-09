"""sensors/collision_sensor.py — Attach a collision sensor to the ego vehicle."""

import carla


class CollisionSensor:
    def __init__(self, world: carla.World, blueprint_library, ego_vehicle: carla.Actor):
        self.collided      = False
        self.collision_evt = None

        bp = blueprint_library.find("sensor.other.collision")
        self._sensor = world.spawn_actor(
            bp,
            carla.Transform(),
            attach_to=ego_vehicle
        )
        self._sensor.listen(self._on_collision)

    def _on_collision(self, event):
        if not self.collided:
            self.collided      = True
            self.collision_evt = event
            other = event.other_actor
            print(f"[COLLISION] t={event.timestamp:.2f}s  "
                  f"other={other.type_id}  "
                  f"impulse={event.normal_impulse}")

    def destroy(self):
        if self._sensor.is_alive:
            self._sensor.stop()
            self._sensor.destroy()
