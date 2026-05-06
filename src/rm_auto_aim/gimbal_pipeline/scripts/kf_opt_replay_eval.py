#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
from typing import Dict, List, Tuple

import numpy as np

from kf_opt_data_loader import load_frames


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Replay-eval tracker params through pybind wrapper")
    p.add_argument("--robot-id", required=True)
    p.add_argument("--log-dir", default="")
    p.add_argument("--obs-csv", default="")
    p.add_argument("--params-json", default="")
    p.add_argument("--module-dir", default="", help="Directory containing gimbal_pipeline_kf_opt*.so")
    p.add_argument("--out-json", default="")
    return p.parse_args()


def _import_module(module_dir: str):
    if module_dir:
        sys.path.insert(0, module_dir)
    try:
        import gimbal_pipeline_kf_opt as m  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "Failed to import gimbal_pipeline_kf_opt. "
            "If you built with ROS humble, use Python 3.10 and pass --module-dir install/gimbal_pipeline/lib. "
            f"Original error: {e}"
        ) from e

    return m


def _snapshot_to_dict(s) -> Dict:
    return {
        "valid": bool(s.valid),
        "timestamp_sec": float(s.timestamp_sec),
        "center": np.array([s.center_x, s.center_y, s.center_z], dtype=float),
        "yaw": float(s.yaw),
        "innov": np.array([s.innov_x, s.innov_y, s.innov_z], dtype=float),
        "nis": float(s.nis),
        "update_type": int(s.update_type),
        "candidate_margin": float(s.candidate_margin),
        "switch_event": int(s.switch_event),
        "binding_confidence": float(s.binding_confidence),
        "degraded_single_obs_mode": bool(s.degraded_single_obs_mode),
    }


def evaluate(frames, wrapper_mod, robot_id: str, overrides: Dict[str, float]) -> Tuple[Dict, float]:
    replay = wrapper_mod.OfflineTrackerReplay()
    replay.set_fixed_robot_id(robot_id)
    replay.reset()
    if overrides:
        replay.apply_numeric_overrides(overrides)

    snapshots: List[Dict] = []
    for f in frames:
        obs_vec = []
        for o in f.observations:
            obs = wrapper_mod.ObservationData()
            obs.x = o["x"]
            obs.y = o["y"]
            obs.z = o["z"]
            obs.yaw = o["yaw"]
            obs.panel_id = o["panel_id"]
            obs.confidence = o["confidence"]
            obs_vec.append(obs)

        replay.feed(f.timestamp_sec, obs_vec)
        s = replay.snapshot()
        snapshots.append(_snapshot_to_dict(s))

    valid = [x for x in snapshots if x["valid"]]
    if not valid:
        report = {"num_frames": len(frames), "num_valid": 0, "loss": 1e9}
        return report, 1e9

    innov_norm = np.array([np.linalg.norm(x["innov"]) for x in valid], dtype=float)
    nis = np.array([x["nis"] for x in valid if x["nis"] >= 0], dtype=float)
    margins = np.array([x["candidate_margin"] for x in valid if not math.isnan(x["candidate_margin"])], dtype=float)
    switches = np.array([x["switch_event"] for x in valid], dtype=int)
    degraded = np.array([1 if x["degraded_single_obs_mode"] else 0 for x in valid], dtype=float)

    # Weak proxy of smoothness using 2nd difference of center trajectory
    centers = np.stack([x["center"] for x in valid], axis=0)
    if centers.shape[0] >= 3:
        jerk = np.diff(centers, n=2, axis=0)
        jerk_norm = np.linalg.norm(jerk, axis=1)
        jerk_p95 = float(np.percentile(jerk_norm, 95))
    else:
        jerk_p95 = 0.0

    nis_p95 = float(np.percentile(nis, 95)) if nis.size > 0 else 1e3
    innov_p95 = float(np.percentile(innov_norm, 95))
    low_margin_ratio = float(np.mean(margins < 0.05)) if margins.size > 0 else 0.5
    switch_rate = float(np.mean(switches > 0))
    degraded_ratio = float(np.mean(degraded)) if degraded.size > 0 else 0.0

    # M2 demo objective (lower is better)
    loss = (
        0.35 * innov_p95
        + 0.20 * (nis_p95 / 100.0)
        + 0.15 * low_margin_ratio
        + 0.10 * switch_rate
        + 0.10 * degraded_ratio
        + 0.10 * jerk_p95
    )

    report = {
        "num_frames": len(frames),
        "num_valid": len(valid),
        "valid_ratio": len(valid) / max(1, len(frames)),
        "innov_p95": innov_p95,
        "nis_p95": nis_p95,
        "low_margin_ratio": low_margin_ratio,
        "switch_rate": switch_rate,
        "degraded_ratio": degraded_ratio,
        "jerk_p95": jerk_p95,
        "loss": loss,
    }
    return report, loss


def main() -> int:
    args = parse_args()
    mod = _import_module(args.module_dir)

    overrides = {}
    if args.params_json:
        with open(args.params_json, "r", encoding="utf-8") as f:
            overrides = json.load(f)

    frames = load_frames(args.obs_csv or None, args.log_dir or None, args.robot_id)
    report, _ = evaluate(frames, mod, args.robot_id, overrides)

    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.out_json:
        with open(args.out_json, "w", encoding="utf-8") as f:
            f.write(text + "\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
