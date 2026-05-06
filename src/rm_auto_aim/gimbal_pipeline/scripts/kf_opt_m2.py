#!/usr/bin/env python3
import argparse
import json
import random
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from kf_opt_data_loader import load_frames
from kf_opt_replay_eval import evaluate


SEARCH_SPACE = {
    "ukf.obs_noise_pos": (0.001, 0.2, "log"),
    "ukf.obs_noise_yaw": (0.001, 0.3, "log"),
    "motion.ca_process_noise_acc": (0.05, 8.0, "log"),
    "motion.process_noise_r": (0.001, 0.2, "log"),
    "motion.process_noise_dz": (0.0005, 0.1, "log"),
    "spin.spin_process_noise_delta_rate": (0.01, 3.0, "log"),
    "spin.spin_process_noise_delta_acc": (0.05, 20.0, "log"),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="M2 parameter optimization with pybind replay")
    p.add_argument("--robot-id", required=True)
    p.add_argument("--log-dir", default="")
    p.add_argument("--obs-csv", default="")
    p.add_argument("--module-dir", default="")
    p.add_argument("--samples", type=int, default=200)
    p.add_argument("--topk", type=int, default=8)
    p.add_argument("--out-dir", default="/tmp/gimbal_kf_opt")
    return p.parse_args()


def import_module(module_dir: str):
    if module_dir:
        sys.path.insert(0, module_dir)
    import gimbal_pipeline_kf_opt as m  # type: ignore

    return m


def sample_params() -> Dict[str, float]:
    out: Dict[str, float] = {}
    for k, (lo, hi, mode) in SEARCH_SPACE.items():
        if mode == "log":
            out[k] = float(np.exp(np.random.uniform(np.log(lo), np.log(hi))))
        else:
            out[k] = random.uniform(lo, hi)
    return out


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    mod = import_module(args.module_dir)
    frames = load_frames(args.obs_csv or None, args.log_dir or None, args.robot_id)
    if not frames:
        raise RuntimeError("No frames found for selected robot_id")

    baseline_report, baseline_loss = evaluate(frames, mod, args.robot_id, {})

    trials: List[Tuple[float, Dict[str, float], Dict]] = []
    for _ in range(args.samples):
        params = sample_params()
        report, loss = evaluate(frames, mod, args.robot_id, params)
        trials.append((loss, params, report))

    trials.sort(key=lambda x: x[0])
    best_loss, best_params, best_report = trials[0]

    summary = {
        "baseline": baseline_report,
        "best": best_report,
        "improvement_ratio": (baseline_loss - best_loss) / max(1e-9, baseline_loss),
        "search_space": SEARCH_SPACE,
        "samples": args.samples,
        "topk": args.topk,
    }

    with open(out_dir / "best_params.json", "w", encoding="utf-8") as f:
        json.dump(best_params, f, indent=2, ensure_ascii=False)
    with open(out_dir / "baseline_report.json", "w", encoding="utf-8") as f:
        json.dump(baseline_report, f, indent=2, ensure_ascii=False)
    with open(out_dir / "opt_report.json", "w", encoding="utf-8") as f:
        json.dump(best_report, f, indent=2, ensure_ascii=False)
    with open(out_dir / "summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
