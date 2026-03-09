"""actors/vehicle_spawner.py — Spawn ego and NPC vehicles in CARLA."""

import random
import carla


EGO_BLUEPRINTS  = ["vehicle.tesla.model3", "vehicle.audi.a2"]
NPC_BLUEPRINTS  = [
    "vehicle.toyota.prius",
    "vehicle.ford.mustang",
    "vehicle.chevrolet.impala",
    "vehicle.mercedes.coupe",
]


class VehicleSpawner:
    def __init__(self, world: carla.World, blueprint_library):
        self.world = world
        self.bpl   = blueprint_library

    # ------------------------------------------------------------------
    def _get_bp(self, names: list):
        for name in names:
            bp = self.bpl.find(name)
            if bp:
                return bp
        # fallback: any car
        return random.choice(self.bpl.filter("vehicle.*"))

    # ------------------------------------------------------------------
    def spawn_ego(self, transform: carla.Transform) -> carla.Actor:
        bp = self._get_bp(EGO_BLUEPRINTS)
        bp.set_attribute("role_name", "ego")

        actor = self.world.try_spawn_actor(bp, transform)
        if actor is None:
            # Jitter spawn location slightly and retry
            for dz in [0.5, 1.0, 1.5, 2.0]:
                t2 = carla.Transform(
                    carla.Location(x=transform.location.x,
                                   y=transform.location.y,
                                   z=transform.location.z + dz),
                    transform.rotation
                )
                actor = self.world.try_spawn_actor(bp, t2)
                if actor:
                    break
        if actor is None:
            raise RuntimeError("Failed to spawn ego vehicle — location may be blocked.")
        return actor

    # ------------------------------------------------------------------
    def spawn_npc(self, transform: carla.Transform) -> carla.Actor:
        bp = self._get_bp(NPC_BLUEPRINTS)
        bp.set_attribute("role_name", "npc")

        actor = self.world.try_spawn_actor(bp, transform)
        if actor is None:
            for dz in [0.5, 1.0, 1.5, 2.0]:
                t2 = carla.Transform(
                    carla.Location(x=transform.location.x,
                                   y=transform.location.y,
                                   z=transform.location.z + dz),
                    transform.rotation
                )
                actor = self.world.try_spawn_actor(bp, t2)
                if actor:
                    break
        if actor is None:
            raise RuntimeError("Failed to spawn NPC vehicle — location may be blocked.")
        return actor
