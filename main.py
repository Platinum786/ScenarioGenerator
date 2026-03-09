"""
CARLA Collision Scenario Engine - main.py
==========================================
Run: python main.py --config configs/scenario.yaml [--sync] [--width 960] [--height 540]
"""

import sys
import os

# Ensure the project root (directory containing main.py) is on sys.path
# so that 'core', 'planner', 'actors', etc. are importable regardless of
# where Python is invoked from.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import argparse
import time
import carla
import pygame
import numpy as np

from core.scenario_parser import ScenarioParser
from core.junction_selector import JunctionSelector
from core.waypoint_extractor import WaypointExtractor
from planner.trajectory_builder import TrajectoryBuilder
from planner.lane_intersection_detector import LaneIntersectionDetector
from planner.spawn_planner import SpawnPlanner
from planner.velocity_planner import VelocityPlanner
from actors.vehicle_spawner import VehicleSpawner
from actors.controller import VehicleController
from sensors.collision_sensor import CollisionSensor
from logging_.data_logger import DataLogger


# =============================================================================
# NPC CONTROL
# =============================================================================

def control_npcs(npcs, world, start_time):
    """
    Drive NPCs toward their target speed using a simple throttle controller.
    This avoids the skidding/rolling caused by enable_constant_velocity
    fighting against the physics engine.
    """
    sim_time = world.get_snapshot().timestamp.elapsed_seconds
    elapsed  = sim_time - start_time

    for npc in npcs:
        actor        = npc["actor"]
        target_speed = npc["speed"]          # m/s

        if elapsed < 1.0:
            # Hold still for the first second
            actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            continue

        current_speed = actor.get_velocity().length()   # m/s
        error         = target_speed - current_speed

        # Simple proportional controller
        if error > 0.5:
            throttle = min(0.8, 0.3 + 0.05 * error)
            brake    = 0.0
        elif error < -1.0:
            throttle = 0.0
            brake    = min(0.5, 0.1 * abs(error))
        else:
            throttle = 0.3          # gentle cruise
            brake    = 0.0

        actor.apply_control(carla.VehicleControl(
            throttle=throttle,
            brake=brake,
            steer=0.0,
            hand_brake=False,
            manual_gear_shift=False,
        ))


# =============================================================================
# MAIN
# =============================================================================

def main():
    argparser = argparse.ArgumentParser(description="CARLA Collision Scenario Engine")
    argparser.add_argument("--config",   default="configs/scenario.yaml", help="Path to scenario YAML")
    argparser.add_argument("--host",     default="127.0.0.1")
    argparser.add_argument("--port",     default=2000, type=int)
    argparser.add_argument("--sync",     action="store_true", help="Enable synchronous mode")
    argparser.add_argument("--width",    default=960,  type=int)
    argparser.add_argument("--height",   default=540,  type=int)
    args = argparser.parse_args()

    # ------------------------------------------------------------------
    # 1. Parse scenario configuration
    # ------------------------------------------------------------------
    parser   = ScenarioParser(args.config)
    scenario = parser.parse()
    print(f"[Config] town={scenario['town']}  junction={scenario['junction_type']}  "
          f"npcs={scenario['npc_count']}  event_time={scenario['event_time']}s  "
          f"type={scenario['collision_type']}")

    # ------------------------------------------------------------------
    # 2. Connect to CARLA
    # ------------------------------------------------------------------
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world  = client.load_world(scenario["town"])
    blueprint_library = world.get_blueprint_library()

    original_settings = world.get_settings()
    if args.sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / 60.0
        world.apply_settings(settings)

    # ------------------------------------------------------------------
    # 3. Junction selection + waypoint extraction
    # ------------------------------------------------------------------
    selector  = JunctionSelector(world, scenario["junction_type"])
    junction  = selector.select()
    print(f"[Junction] id={junction.id}  center≈{selector.center(junction)}")

    extractor    = WaypointExtractor(junction)
    traj_builder = TrajectoryBuilder()
    trajectories = []
    for entry_wp, exit_wp in extractor.get_entry_exit_pairs():
        traj = traj_builder.build(entry_wp, exit_wp)
        if traj:
            trajectories.append(traj)

    print(f"[Trajectories] {len(trajectories)} trajectories found through junction")
    if len(trajectories) < 2:
        raise RuntimeError("Not enough trajectories to plan a collision. Try a different town/junction.")

    # ------------------------------------------------------------------
    # 4. Lane intersection detection
    # ------------------------------------------------------------------
    detector = LaneIntersectionDetector(threshold=2.5)
    collision_point, traj_pair = detector.find_intersection(trajectories)
    if collision_point is None:
        raise RuntimeError("No lane intersection found. Increase threshold or use 4-way junction.")
    print(f"[Collision Point] x={collision_point.x:.2f}  y={collision_point.y:.2f}  z={collision_point.z:.2f}")

    # ------------------------------------------------------------------
    # 5. Spawn point planning
    # ------------------------------------------------------------------
    sp_planner = SpawnPlanner(spawn_distance=50.0)
    spawn_data = sp_planner.plan(traj_pair, collision_point, scenario["npc_count"])
    # spawn_data: list of {"transform": carla.Transform, "distance": float}

    # ------------------------------------------------------------------
    # 6. Velocity planning
    # ------------------------------------------------------------------
    vel_planner = VelocityPlanner(scenario["event_time"])
    for sd in spawn_data:
        sd["speed"]    = vel_planner.compute(sd["distance"])
        sd["velocity"] = vel_planner.velocity_vector(sd["transform"], sd["speed"])
        print(f"  spawn dist={sd['distance']:.1f}m  speed={sd['speed']:.2f}m/s  "
              f"({sd['speed']*3.6:.1f} km/h)")

    # ------------------------------------------------------------------
    # 7. Spawn vehicles
    # ------------------------------------------------------------------
    spawner = VehicleSpawner(world, blueprint_library)
    ego_vehicle = spawner.spawn_ego(spawn_data[0]["transform"])
    print(f"[Spawned] ego vehicle id={ego_vehicle.id}")

    npcs = []
    for npc_idx, sd in enumerate(spawn_data[1:]):
        npc_actor = spawner.spawn_npc(sd["transform"])
        npcs.append({"actor": npc_actor, "speed": sd["speed"]})
        print(f"[Spawned] NPC id={npc_actor.id}  speed={sd['speed']:.2f}m/s")

    # Enable autopilot on ego — Traffic Manager drives it naturally
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(args.sync)
    traffic_manager.set_desired_speed(ego_vehicle, spawn_data[0]["speed"] * 3.6)  # km/h
    traffic_manager.ignore_lights_percentage(ego_vehicle, 100)
    traffic_manager.ignore_signs_percentage(ego_vehicle, 100)
    ego_vehicle.set_autopilot(True, traffic_manager.get_port())
    print(f"[Ego] Autopilot ON  target={spawn_data[0]['speed']*3.6:.1f} km/h")

    # Only NPCs are controlled manually; ego is handled by autopilot
    npcs_all = npcs

    # ------------------------------------------------------------------
    # 8. Collision sensor + logger
    # ------------------------------------------------------------------
    col_sensor = CollisionSensor(world, blueprint_library, ego_vehicle)
    logger     = DataLogger("logs/scenario_log.json")

    # ------------------------------------------------------------------
    # 9. Pygame display
    # ------------------------------------------------------------------
    pygame.init()
    display = pygame.display.set_mode(
        (args.width * 2, args.height),
        pygame.HWSURFACE | pygame.DOUBLEBUF
    )
    pygame.display.set_caption("CARLA Collision Scenario — Drone | Ego")
    clock = pygame.time.Clock()
    font  = pygame.font.SysFont("monospace", 16)

    # =========================
    # CAMERA SETUP
    # =========================
    shared_data = {"ego_surface": None, "drone_surface": None}

    def process_image(image, key):
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        shared_data[key] = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    cam_bp = blueprint_library.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(args.width))
    cam_bp.set_attribute("image_size_y", str(args.height))

    ego_cam = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(x=-6, z=2.5),
                        carla.Rotation(pitch=-15)),
        attach_to=ego_vehicle
    )
    ego_cam.listen(lambda image: process_image(image, "ego_surface"))

    drone_cam = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(z=80),
                        carla.Rotation(pitch=-90))
    )
    drone_cam.listen(lambda image: process_image(image, "drone_surface"))

    start_time = None

    try:
        while True:
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # Update drone to follow ego
            ego_loc = ego_vehicle.get_location()
            drone_cam.set_transform(
                carla.Transform(
                    carla.Location(x=ego_loc.x, y=ego_loc.y, z=80),
                    carla.Rotation(pitch=-90)
                )
            )

            # Rendering
            if shared_data["drone_surface"]:
                display.blit(shared_data["drone_surface"], (0, 0))
            if shared_data["ego_surface"]:
                display.blit(shared_data["ego_surface"], (args.width, 0))

            # HUD overlay
            sim_time = world.get_snapshot().timestamp.elapsed_seconds
            if start_time is None:
                start_time = sim_time
            elapsed = sim_time - start_time

            hud_lines = [
                f"Elapsed : {elapsed:.1f}s / {scenario['event_time']}s",
                f"Ego spd : {ego_vehicle.get_velocity().length()*3.6:.1f} km/h",
                f"Collided: {col_sensor.collided}",
            ]
            for i, line in enumerate(hud_lines):
                surf = font.render(line, True, (255, 255, 0))
                display.blit(surf, (10, 10 + i * 20))

            pygame.display.flip()
            clock.tick(60)

            # Control all vehicles
            control_npcs(npcs_all, world, start_time)

            # Log collision event
            if col_sensor.collided and not logger.logged:
                logger.log_collision(
                    timestamp=elapsed,
                    ego_location=ego_vehicle.get_location(),
                    collision_point=collision_point,
                    scenario=scenario,
                )

            # Stop after event_time + 5s buffer
            if elapsed > scenario["event_time"] + 5:
                print("[Scenario] Complete — exiting.")
                break

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

    finally:
        col_sensor.destroy()
        ego_cam.stop()
        drone_cam.stop()
        ego_cam.destroy()
        drone_cam.destroy()
        for npc_data in npcs:
            npc_data["actor"].destroy()
        ego_vehicle.destroy()
        world.apply_settings(original_settings)
        pygame.quit()
        logger.save()
        print("[Done] Log saved.")


if __name__ == "__main__":
    main()