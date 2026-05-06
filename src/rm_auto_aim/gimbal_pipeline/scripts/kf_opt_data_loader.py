#!/usr/bin/env python3
import glob
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import pandas as pd


@dataclass
class FrameBatch:
    timestamp_sec: float
    observations: List[Dict]


def _latest_obs_csv(log_dir: str) -> str:
    files = sorted(glob.glob(os.path.join(log_dir, "observation_log_*.csv")))
    if not files:
        raise FileNotFoundError(f"No observation_log_*.csv found in {log_dir}")
    return files[-1]


def load_frames(obs_csv: Optional[str], log_dir: Optional[str], robot_id: str) -> List[FrameBatch]:
    if not obs_csv:
        if not log_dir:
            raise ValueError("Either obs_csv or log_dir must be provided")
        obs_csv = _latest_obs_csv(log_dir)

    df = pd.read_csv(obs_csv)
    if "robot_id" not in df.columns:
        raise ValueError("observation csv missing robot_id column")

    df = df[df["robot_id"] == robot_id].copy()
    if df.empty:
        return []

    df = df.sort_values("timestamp_ns")
    grouped = df.groupby("timestamp_ns", sort=True)

    frames: List[FrameBatch] = []
    for ts_ns, g in grouped:
        obs_list: List[Dict] = []
        for _, row in g.iterrows():
            panel_id = int(row["panel_id"]) if pd.notna(row.get("panel_id")) else None
            obs_list.append(
                {
                    "x": float(row["obs_x"]),
                    "y": float(row["obs_y"]),
                    "z": float(row["obs_z"]),
                    "yaw": float(row["obs_yaw"]),
                    "panel_id": panel_id,
                    "confidence": float(row.get("confidence", 1.0)),
                }
            )
        frames.append(FrameBatch(timestamp_sec=float(ts_ns) * 1e-9, observations=obs_list))

    return frames
