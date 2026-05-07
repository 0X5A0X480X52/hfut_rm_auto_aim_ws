from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.config import Config
from src.simulation import run_simulation_with_projection, summarize
from src.visualization import plot_armor_plane_95, plot_projection_trajectory, plot_results


def main() -> int:
    parser = argparse.ArgumentParser(description="Probability ballistic fire advice demo")
    parser.add_argument("--config", default=str(ROOT / "config" / "default_config.yaml"))
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    config = Config.load(args.config)
    rows, projection = run_simulation_with_projection(config)
    summary = summarize(rows)
    viz_cfg = config.section("visualization")
    output_dir = ROOT / viz_cfg.get("output_dir", "outputs")
    if not args.no_plot:
        figure = plot_results(rows, output_dir / viz_cfg.get("figure_name", "fire_probability_demo.png"), bool(viz_cfg.get("show", False)))
        plane_figure = plot_armor_plane_95(rows, output_dir / "armor_plane_projection_95.png", bool(viz_cfg.get("show", False)))
        traj_figure = plot_projection_trajectory(
            projection,
            output_dir / "armor_plane_projection_trajectory.png",
            bool(viz_cfg.get("show", False)),
        )
        print(f"figure: {figure}")
        print(f"armor_plane_95: {plane_figure}")
        print(f"armor_plane_trajectory: {traj_figure}")
    print("summary:")
    for key, value in summary.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.6f}")
        else:
            print(f"  {key}: {value}")
    print("first frame:")
    for key in ("p_hit_window", "fire_score", "fire_advice", "elapsed_us", "best_tau_ms", "e_u", "e_v", "sigma_u", "sigma_v"):
        print(f"  {key}: {rows[0][key]:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
