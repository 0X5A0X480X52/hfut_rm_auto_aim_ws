from __future__ import annotations

import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.config import Config
from src.simulation import run_simulation, summarize


def run_mode(base: Config, sigma_enabled: bool):
    raw = copy.deepcopy(base.raw)
    raw["sigma_point_extension"]["enable"] = sigma_enabled
    cfg = Config(raw)
    return summarize(run_simulation(cfg))


def main() -> int:
    cfg = Config.load(ROOT / "config" / "default_config.yaml")
    for name, enabled in (("base", False), ("sigma_point", True)):
        summary = run_mode(cfg, enabled)
        print(f"{name}:")
        for key in ("elapsed_mean_us", "elapsed_p95_us", "elapsed_max_us", "p_hit_max", "fire_advice_ratio"):
            print(f"  {key}: {summary[key]:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
