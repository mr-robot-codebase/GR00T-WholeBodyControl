"""Load robot asset paths from robot_paths.yaml at the repo root."""

from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_CONFIG_FILE = _REPO_ROOT / "robot_paths.yaml"

_cache: dict | None = None


def _load() -> dict:
    global _cache
    if _cache is None:
        with open(_CONFIG_FILE) as f:
            _cache = yaml.safe_load(f)
    return _cache


def get_robot_paths(robot: str) -> dict[str, Path]:
    """Return absolute paths for a robot from robot_paths.yaml.

    Keys match the YAML (urdf, asset_path, scene, meshes, mjcf, ...).
    """
    config = _load()
    robots = config.get("robots", {})
    if robot not in robots:
        available = list(robots.keys())
        raise KeyError(f"Robot '{robot}' not in robot_paths.yaml. Available: {available}")
    return {k: _REPO_ROOT / v for k, v in robots[robot].items()}
