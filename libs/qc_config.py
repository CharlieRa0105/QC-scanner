"""
qc_config.py

One place to load the QC Scanner's YAML configuration.

There are two config files (architecture.md §6):

  * config/system_config.yaml -- shared, cell-independent tunables (TRACKED).
  * config/local_config.yaml  -- site-specific, MEASURED values for one cell
                                  (robot IP, the marked-corner calibration);
                                  git-ignored, may be absent.

`load_config()` reads the shared file and deep-merges the local file on top of
it (local wins), returning a plain nested dict. Keeping this in one small module
means every consumer -- the plan_path.py / scanpath_convert.py CLIs today, the
PathPlanner node later -- reads config the same way instead of each re-deriving
paths and merge rules.

Dependencies: PyYAML (the only non-stdlib dep of the planner-side tooling).
"""

from pathlib import Path

import yaml

# config/ sits at the repo root, one level up from libs/.
_REPO_ROOT = Path(__file__).resolve().parent.parent
_CONFIG_DIR = _REPO_ROOT / "config"


def _deep_merge(base, override):
    """Recursively merge `override` into `base` and return `base`.

    Nested dicts merge key by key; any non-dict value (including lists) is
    replaced wholesale by the override. Used so local_config.yaml can override
    just the keys it cares about without having to restate the shared file.
    """
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(config_dir=None):
    """
    Load and merge the QC Scanner configuration.

    Args:
        config_dir: directory holding the YAML files. Defaults to the repo's
            config/ directory (so callers don't hard-code the path).

    Returns:
        A nested dict: system_config.yaml with local_config.yaml merged on top.

    Raises:
        FileNotFoundError: if system_config.yaml is missing (that one is
            required and version-controlled). A missing local_config.yaml is
            tolerated -- the shared defaults are returned unchanged.
    """
    config_dir = Path(config_dir) if config_dir else _CONFIG_DIR

    system_path = config_dir / "system_config.yaml"
    if not system_path.is_file():
        raise FileNotFoundError(
            f"required shared config not found: {system_path} "
            "(it is tracked in git -- see architecture.md §6)"
        )
    with open(system_path) as f:
        config = yaml.safe_load(f) or {}

    local_path = config_dir / "local_config.yaml"
    if local_path.is_file():
        with open(local_path) as f:
            local = yaml.safe_load(f) or {}
        _deep_merge(config, local)

    return config
