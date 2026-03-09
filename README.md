# CARLA Collision Scenario Engine

Automatically generates controlled collision scenarios in the CARLA simulator
based on a YAML configuration file.

## Quick Start

```bash
# 1. Install dependencies (match carla version to your server)
pip install -r requirements.txt

# 2. Start CARLA server
./CarlaUE4.sh -quality-level=Low          # Linux
#for us
sudo docker run -p 2000-2002:2000-2002 --gpus "device=0" carlasim/carla:0.9.15 /bin/bash ./CarlaUE4.sh -RenderOffScreen -quality-level=Epic



# 3. Run the scenario (synchronous mode recommended)
 python main.py --config configs/scenario.yaml --sync --host 172.27.21.11 --port 2000
```

## Configuration (`configs/scenario.yaml`)

| Field           | Values                          | Description                          |
|-----------------|---------------------------------|--------------------------------------|
| `town`          | Town01–Town12                   | CARLA map to load                    |
| `junction_type` | `3_way` / `4_way`               | Type of junction to use              |
| `npc_count`     | 1–3                             | Number of NPC vehicles (+ 1 ego)     |
| `event_time`    | seconds (e.g. `8`)              | Time until collision from t=0        |
| `collision_type`| `t_bone` / `head_on` / `rear_end` | Intended collision geometry        |

## Display Layout

```
┌────────────────────┬────────────────────┐
│  Drone cam (top)   │   Ego cam (chase)  │
│  follows ego       │   behind vehicle   │
└────────────────────┴────────────────────┘
```

HUD (top-left) shows elapsed time, ego speed, and collision status.

## Timeline

```
t=0s  → All vehicles spawned at computed positions
t=1s  → Constant-velocity motion begins
t=Ns  → Vehicles reach intersection and collide  (N = event_time)
t=N+5s→ Scenario ends, log saved to logs/scenario_log.json
```

## Project Structure

```
scenario_engine/
├── configs/scenario.yaml       # User configuration
├── core/
│   ├── scenario_parser.py      # Load & validate YAML
│   ├── junction_selector.py    # Find 3-way or 4-way junction
│   └── waypoint_extractor.py   # Entry-exit pairs from junction
├── planner/
│   ├── trajectory_builder.py   # Walk waypoints entry→exit
│   ├── lane_intersection_detector.py  # Find crossing point
│   ├── spawn_planner.py        # Position vehicles ~50m before collision
│   └── velocity_planner.py     # speed = distance / event_time
├── actors/
│   ├── vehicle_spawner.py      # Spawn ego + NPC blueprints
│   └── controller.py           # Constant-velocity wrapper
├── sensors/
│   └── collision_sensor.py     # Detect & log impact event
├── logging_/
│   └── data_logger.py          # Save JSON report
├── logs/                       # Auto-created output directory
├── main.py                     # Entry point
└── requirements.txt
```

## CLI Options

```
--config    Path to scenario YAML (default: configs/scenario.yaml)
--host      CARLA server host (default: 127.0.0.1)
--port      CARLA server port (default: 2000)
--sync      Enable synchronous simulation mode (recommended)
--width     Camera width in pixels (default: 960)
--height    Camera height in pixels (default: 540)
```
