"""logging_/data_logger.py — Record scenario outcome to JSON."""

import json
import os


class DataLogger:
    def __init__(self, path: str):
        self.path   = path
        self.logged = False
        self._data  = {}
        os.makedirs(os.path.dirname(path), exist_ok=True)

    def log_collision(self, timestamp: float, ego_location, collision_point, scenario: dict):
        self._data = {
            "collision_detected": True,
            "elapsed_time_s":     round(timestamp, 3),
            "ego_location": {
                "x": round(ego_location.x, 3),
                "y": round(ego_location.y, 3),
                "z": round(ego_location.z, 3),
            },
            "planned_collision_point": {
                "x": round(collision_point.x, 3),
                "y": round(collision_point.y, 3),
                "z": round(collision_point.z, 3),
            },
            "scenario": scenario,
        }
        self.logged = True
        print(f"[Logger] Collision event recorded at t={timestamp:.2f}s")

    def save(self):
        if not self._data:
            self._data = {"collision_detected": False}
        with open(self.path, "w") as f:
            json.dump(self._data, f, indent=2)
        print(f"[Logger] Saved → {self.path}")
