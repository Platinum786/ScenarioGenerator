"""core/scenario_parser.py — Load and validate the YAML scenario config."""

import yaml
import os


VALID_JUNCTION_TYPES = {"3_way", "4_way"}
VALID_COLLISION_TYPES = {"t_bone", "head_on", "rear_end"}


class ScenarioParser:
    def __init__(self, config_path: str):
        self.config_path = config_path

    def parse(self) -> dict:
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config not found: {self.config_path}")

        with open(self.config_path, "r") as f:
            raw = yaml.safe_load(f)

        cfg = raw.get("scenario", {})

        # Validate / apply defaults
        town           = cfg.get("town", "Town05")
        junction_type  = cfg.get("junction_type", "4_way")
        npc_count      = int(cfg.get("npc_count", 1))
        event_time     = float(cfg.get("event_time", 8.0))
        collision_type = cfg.get("collision_type", "t_bone")

        if junction_type not in VALID_JUNCTION_TYPES:
            raise ValueError(f"junction_type must be one of {VALID_JUNCTION_TYPES}")
        if collision_type not in VALID_COLLISION_TYPES:
            raise ValueError(f"collision_type must be one of {VALID_COLLISION_TYPES}")
        if npc_count < 1:
            raise ValueError("npc_count must be >= 1")
        if event_time <= 0:
            raise ValueError("event_time must be positive")

        return {
            "town":           town,
            "junction_type":  junction_type,
            "npc_count":      npc_count,
            "event_time":     event_time,
            "collision_type": collision_type,
        }
