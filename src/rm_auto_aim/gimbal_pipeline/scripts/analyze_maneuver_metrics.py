#!/usr/bin/env python3
"""
Analyze maneuver-detection metrics from tracker_state_log_*.csv.

Key outputs:
1) Single-indicator threshold scan (recall/FPR/precision/F1)
2) Split-threshold combo rule search for better recall-vs-FPR tradeoff
3) Recommended thresholds for low false-positive operation

Default labeling mode is pseudo labels built from kinematic shocks:
- linear acceleration spike from center velocity derivative
- yaw acceleration spike from yaw_velocity derivative

Usage example:
  python3 analyze_maneuver_metrics.py \
    --log_dir /home/amatrix02/hfut_rm_auto_aim_ws/tmp/prediction_logs \
    --fpr_target 0.10
"""

from __future__ import annotations

import argparse
import glob
import json
import os
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


@dataclass
class EvalRow:
    threshold: float
    quantile: float
    recall: float
    fpr: float
    precision: float
    f1: float
    tp: int
    fp: int
    fn: int
    tn: int


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Maneuver metric analysis from tracker_state logs")
    p.add_argument("--log_dir", required=True, help="Directory containing tracker_state_log_*.csv")
    p.add_argument("--state_csv", default="", help="Optional explicit tracker_state CSV path")
    p.add_argument("--fpr_target", type=float, default=0.10, help="FPR constraint for recommendation")
    p.add_argument("--quantile_min", type=float, default=0.70, help="Min quantile for threshold sweep")
    p.add_argument("--quantile_max", type=float, default=0.995, help="Max quantile for threshold sweep")
    p.add_argument("--quantile_steps", type=int, default=80, help="Threshold sweep steps")
    p.add_argument("--event_q", type=float, default=0.97, help="Quantile for pseudo-event construction")
    p.add_argument("--event_dilate", type=int, default=2, help="Window size to dilate pseudo events (+/- frames)")
    p.add_argument("--label_column", default="", help="Optional ground-truth label column in CSV (0/1)")
    p.add_argument("--output_dir", default="", help="Directory for output summary files")
    p.add_argument("--filter_mad", action="store_true", help="Apply MAD-based rolling outlier filter to nis and innov_norm before analysis")
    p.add_argument("--mad_window", type=int, default=10, help="Window size for MAD filter")
    p.add_argument("--mad_k", type=float, default=3.0, help="MAD multiplier for outlier detection")
    return p.parse_args()


def find_latest_state_csv(log_dir: str) -> str:
    files = sorted(glob.glob(os.path.join(log_dir, "tracker_state_log_*.csv")))
    if not files:
        raise FileNotFoundError(f"No tracker_state_log_*.csv found in {log_dir}")
    return files[-1]


def safe_dt_from_ns(ts_ns: np.ndarray) -> np.ndarray:
    t = ts_ns.astype(float) * 1e-9
    dt = np.diff(t, prepend=t[0])
    positive = dt[dt > 1e-6]
    fallback = float(np.median(positive)) if positive.size else 0.01
    dt[dt <= 1e-6] = fallback
    return dt


def filter_outliers_mad(data: np.ndarray, window: int = 10, k: float = 3.0) -> np.ndarray:
    """Simple rolling MAD filter that replaces outliers with the moving median.

    ``window`` specifies the number of past samples (including current) to
    consider.  ``k`` is the multiplier applied to the median absolute
    deviation.  When MAD is zero the raw value is kept unchanged.
    """
    filtered = []
    for i in range(len(data)):
        start = max(0, i - window + 1)
        window_data = data[start : i + 1]
        med = np.median(window_data)
        mad = np.median(np.abs(window_data - med))
        if mad == 0:
            filtered.append(data[i])
        elif abs(data[i] - med) > k * mad:
            filtered.append(med)
        else:
            filtered.append(data[i])
    return np.array(filtered)


def build_pseudo_events(df: pd.DataFrame, event_q: float, event_dilate: int) -> Tuple[np.ndarray, Dict[str, float]]:
    dt = safe_dt_from_ns(df["timestamp_ns"].to_numpy())
    vel = df[["vel_x", "vel_y", "vel_z"]].to_numpy(dtype=float)
    acc = np.vstack([np.zeros(3), np.diff(vel, axis=0)]) / dt[:, None]
    acc_mag = np.linalg.norm(acc, axis=1)

    yaw_v = df["yaw_velocity"].to_numpy(dtype=float)
    yaw_acc = np.r_[0.0, np.diff(yaw_v)] / dt

    thr_acc = float(np.quantile(acc_mag, event_q))
    thr_yaw_acc = float(np.quantile(np.abs(yaw_acc), event_q))

    event = ((acc_mag > thr_acc) | (np.abs(yaw_acc) > thr_yaw_acc)).astype(np.int32)

    for k in range(1, max(event_dilate, 0) + 1):
        event = np.maximum(event, np.r_[event[k:], np.zeros(k, dtype=np.int32)])
        event = np.maximum(event, np.r_[np.zeros(k, dtype=np.int32), event[:-k]])

    meta = {
        "event_q": event_q,
        "event_dilate": event_dilate,
        "thr_acc": thr_acc,
        "thr_yaw_acc": thr_yaw_acc,
        "event_ratio": float(event.mean()),
    }
    return event, meta


def confusion_metrics(pred: np.ndarray, y: np.ndarray) -> EvalRow:
    tp = int(np.sum((pred == 1) & (y == 1)))
    fp = int(np.sum((pred == 1) & (y == 0)))
    fn = int(np.sum((pred == 0) & (y == 1)))
    tn = int(np.sum((pred == 0) & (y == 0)))

    recall = tp / (tp + fn + 1e-12)
    fpr = fp / (fp + tn + 1e-12)
    precision = tp / (tp + fp + 1e-12)
    f1 = 2.0 * precision * recall / (precision + recall + 1e-12)
    return EvalRow(
        threshold=float("nan"),
        quantile=float("nan"),
        recall=float(recall),
        fpr=float(fpr),
        precision=float(precision),
        f1=float(f1),
        tp=tp,
        fp=fp,
        fn=fn,
        tn=tn,
    )


def scan_thresholds(x: np.ndarray, y: np.ndarray, qmin: float, qmax: float, qsteps: int) -> Dict[str, EvalRow]:
    mask = np.isfinite(x)
    x = x[mask]
    y = y[mask]
    if x.size == 0:
        raise ValueError("No finite samples for indicator")

    quantiles = np.linspace(qmin, qmax, qsteps)
    rows = []
    for q in quantiles:
        th = float(np.quantile(x, q))
        pred = (x > th).astype(np.int32)
        row = confusion_metrics(pred, y)
        row.threshold = th
        row.quantile = float(q)
        rows.append(row)

    best_score = max(rows, key=lambda r: r.recall - 0.8 * r.fpr)
    best_f1 = max(rows, key=lambda r: r.f1)

    return {
        "best_score": best_score,
        "best_f1": best_f1,
        "rows": rows,
    }


def best_under_fpr(rows: list[EvalRow], fpr_target: float) -> EvalRow | None:
    feasible = [r for r in rows if r.fpr <= fpr_target]
    if not feasible:
        return None
    return max(feasible, key=lambda r: r.recall)


def analyze_indicators(df: pd.DataFrame, y: np.ndarray, args: argparse.Namespace) -> Dict[str, dict]:
    # if filtering produced an explicit innov_norm column then use it;
    # otherwise compute from components on the fly (as before).
    if "innov_norm" in df.columns:
        innov_arr = df["innov_norm"].to_numpy(dtype=float)
    else:
        innov_arr = np.linalg.norm(df[["innov_x", "innov_y", "innov_z"]].to_numpy(dtype=float), axis=1)

    indicators = {
        "nis": df["nis"].to_numpy(dtype=float),
        "innov_norm": innov_arr,
        "innov_yaw_abs": np.abs(df["innov_yaw"].to_numpy(dtype=float)),
        "accel_magnitude": df["accel_magnitude"].to_numpy(dtype=float),
        "yaw_vel_abs": np.abs(df["yaw_velocity"].to_numpy(dtype=float)),
    }

    out = {}
    for name, x in indicators.items():
        scanned = scan_thresholds(x, y, args.quantile_min, args.quantile_max, args.quantile_steps)
        out[name] = {
            "best_score": scanned["best_score"],
            "best_f1": scanned["best_f1"],
            "best_recall_under_fpr_target": best_under_fpr(scanned["rows"], args.fpr_target),
        }

    # Stratified NIS by update_type
    ut = df["update_type"].to_numpy(dtype=int)
    for u in [1, 2]:
        mask = ut == u
        if int(mask.sum()) < 50:
            continue
        scanned = scan_thresholds(
            df.loc[mask, "nis"].to_numpy(dtype=float),
            y[mask],
            args.quantile_min,
            args.quantile_max,
            args.quantile_steps,
        )
        out[f"nis_update_type_{u}"] = {
            "best_score": scanned["best_score"],
            "best_f1": scanned["best_f1"],
            "best_recall_under_fpr_target": best_under_fpr(scanned["rows"], args.fpr_target),
        }

    return out


def search_combo_rule(df: pd.DataFrame, y: np.ndarray, fpr_target: float) -> Dict[str, object]:
    ut = df["update_type"].to_numpy(dtype=int)
    nis = df["nis"].to_numpy(dtype=float)
    inn = np.linalg.norm(df[["innov_x", "innov_y", "innov_z"]].to_numpy(dtype=float), axis=1)

    q_nis = np.linspace(0.75, 0.98, 15)
    q_inn = [0.80, 0.85, 0.90, 0.93, 0.95]

    best = None
    best_low_fp = None

    for q1 in q_nis:
        nis1_vals = nis[ut == 1]
        if nis1_vals.size == 0:
            continue
        t1 = float(np.quantile(nis1_vals, q1))
        for q2 in q_nis:
            nis2_vals = nis[ut == 2]
            if nis2_vals.size == 0:
                continue
            t2 = float(np.quantile(nis2_vals, q2))
            for qi1 in q_inn:
                inn1_vals = inn[ut == 1]
                if inn1_vals.size == 0:
                    continue
                ti1 = float(np.quantile(inn1_vals, qi1))
                for qi2 in q_inn:
                    inn2_vals = inn[ut == 2]
                    if inn2_vals.size == 0:
                        continue
                    ti2 = float(np.quantile(inn2_vals, qi2))

                    pred = (((ut == 1) & (nis > t1) & (inn > ti1)) |
                            ((ut == 2) & (nis > t2) & (inn > ti2))).astype(np.int32)
                    m = confusion_metrics(pred, y)

                    candidate = {
                        "q_nis_1": float(q1),
                        "q_nis_2": float(q2),
                        "q_inn_1": float(qi1),
                        "q_inn_2": float(qi2),
                        "th_nis_1": t1,
                        "th_nis_2": t2,
                        "th_inn_1": ti1,
                        "th_inn_2": ti2,
                        "recall": m.recall,
                        "fpr": m.fpr,
                        "precision": m.precision,
                        "f1": m.f1,
                        "tp": m.tp,
                        "fp": m.fp,
                    }

                    score = m.recall - 1.2 * m.fpr
                    if best is None or score > best["score"]:
                        best = {"score": float(score), **candidate}

                    if m.fpr <= fpr_target:
                        if best_low_fp is None or m.recall > best_low_fp["recall"]:
                            best_low_fp = candidate

    return {
        "best_tradeoff": best,
        "best_recall_under_fpr_target": best_low_fp,
    }


def evalrow_to_dict(row: EvalRow | None) -> dict | None:
    if row is None:
        return None
    return {
        "threshold": row.threshold,
        "quantile": row.quantile,
        "recall": row.recall,
        "fpr": row.fpr,
        "precision": row.precision,
        "f1": row.f1,
        "tp": row.tp,
        "fp": row.fp,
        "fn": row.fn,
        "tn": row.tn,
    }


def main() -> int:
    args = parse_args()

    state_csv = args.state_csv or find_latest_state_csv(args.log_dir)
    df = pd.read_csv(state_csv).sort_values("timestamp_ns").reset_index(drop=True)

    # optionally filter nis/innov_norm for outliers prior to anything else
    if args.filter_mad:
        # filter nis separately for each update_type, preserving unclassified rows
        if "nis" in df.columns and "update_type" in df.columns:
            nis_arr = df["nis"].to_numpy(dtype=float)
            ut_arr = df["update_type"].to_numpy(dtype=int)
            filtered_nis = nis_arr.copy()
            for u in np.unique(ut_arr):
                mask = ut_arr == u
                if mask.any():
                    filtered_nis[mask] = filter_outliers_mad(nis_arr[mask], window=args.mad_window, k=args.mad_k)
            df["nis"] = filtered_nis
        elif "nis" in df.columns:
            # fallback if no update_type column
            df["nis"] = filter_outliers_mad(df["nis"].to_numpy(dtype=float), window=args.mad_window, k=args.mad_k)
        # compute innov_norm and filter it, store back to column (overwrite if exists)
        innov = np.linalg.norm(df[["innov_x", "innov_y", "innov_z"]].to_numpy(dtype=float), axis=1)
        df["innov_norm"] = filter_outliers_mad(innov, window=args.mad_window, k=args.mad_k)

    if args.label_column:
        if args.label_column not in df.columns:
            raise KeyError(f"label_column '{args.label_column}' not found in CSV")
        y = (df[args.label_column].to_numpy(dtype=float) > 0.5).astype(np.int32)
        label_meta = {"label_mode": "ground_truth", "label_column": args.label_column}
    else:
        y, pseudo_meta = build_pseudo_events(df, args.event_q, args.event_dilate)
        label_meta = {"label_mode": "pseudo", **pseudo_meta}

    basic = {
        "state_csv": state_csv,
        "rows": int(len(df)),
        "robots": int(df["robot_id"].nunique()) if "robot_id" in df.columns else 0,
        "update_type_counts": {str(k): int(v) for k, v in df["update_type"].value_counts(dropna=False).to_dict().items()},
    }

    indicator_res = analyze_indicators(df, y, args)
    combo_res = search_combo_rule(df, y, args.fpr_target)

    result = {
        "basic": basic,
        "label": label_meta,
        "fpr_target": args.fpr_target,
        "indicators": {
            k: {
                "best_score": evalrow_to_dict(v["best_score"]),
                "best_f1": evalrow_to_dict(v["best_f1"]),
                "best_recall_under_fpr_target": evalrow_to_dict(v["best_recall_under_fpr_target"]),
            }
            for k, v in indicator_res.items()
        },
        "combo_rule": combo_res,
    }

    out_dir = args.output_dir or args.log_dir
    os.makedirs(out_dir, exist_ok=True)
    out_json = os.path.join(out_dir, "maneuver_metric_report.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    print("== Maneuver Metric Analysis ==")
    print(f"state_csv: {state_csv}")
    print(f"rows: {basic['rows']}, robots: {basic['robots']}")
    print(f"label_mode: {label_meta['label_mode']}")
    if label_meta["label_mode"] == "pseudo":
        print(
            "pseudo_event: ratio={:.3f}, thr_acc={:.3f}, thr_yaw_acc={:.3f}".format(
                label_meta["event_ratio"], label_meta["thr_acc"], label_meta["thr_yaw_acc"]
            )
        )
    print(f"fpr_target: {args.fpr_target:.3f}")
    print("\nTop recommendations (single indicators, under fpr_target):")
    for k, v in result["indicators"].items():
        row = v["best_recall_under_fpr_target"]
        if row is None:
            print(f"- {k}: no threshold meets fpr_target")
            continue
        print(
            "- {}: th={:.6g}, rec={:.3f}, fpr={:.3f}, prec={:.3f}, f1={:.3f}".format(
                k, row["threshold"], row["recall"], row["fpr"], row["precision"], row["f1"]
            )
        )

    print("\nCombo rule recommendation:")
    c = result["combo_rule"]["best_recall_under_fpr_target"]
    if c is None:
        print("- no combo threshold satisfies fpr_target")
    else:
        print(
            "- if update_type=1: nis>{:.3f} and innov_norm>{:.4f}; "
            "if update_type=2: nis>{:.3f} and innov_norm>{:.4f}; "
            "rec={:.3f}, fpr={:.3f}, prec={:.3f}, f1={:.3f}".format(
                c["th_nis_1"], c["th_inn_1"], c["th_nis_2"], c["th_inn_2"],
                c["recall"], c["fpr"], c["precision"], c["f1"]
            )
        )

    print(f"\nSaved report: {out_json}")
    # generate time-series visualizations for NIS and innov_norm
    try:
        plot_time_series(df, out_dir)
    except Exception as e:
        print(f"Warning: failed to produce plots: {e}")
    return 0


def plot_time_series(df: pd.DataFrame, out_dir: str) -> None:
    """Create vertical plots of nis (by update_type) and innov_norm versus time.

    The first two subplots show nis for update_type 1 and 2 separately.  Percentile
    lines (P25, P50, P75, P90, P95) are overlaid with dashed lines and annotated in the
    legend.  The third subplot shows the innovation norm with its own percentile lines.
    All plots share a common time axis in seconds (relative to the start of the log).
    """
    # convert timestamp to seconds relative to start
    t_ns = df["timestamp_ns"].to_numpy(dtype=float)
    if t_ns.size == 0:
        return
    t = (t_ns - t_ns[0]) * 1e-9

    nis = df.get("nis").to_numpy(dtype=float) if "nis" in df.columns else np.zeros_like(t)
    innov_norm = np.linalg.norm(df[["innov_x", "innov_y", "innov_z"]].to_numpy(dtype=float), axis=1)
    ut = df.get("update_type").to_numpy(dtype=int) if "update_type" in df.columns else np.zeros_like(t, dtype=int)

    def quantile_lines(arr: np.ndarray):
        qs = [0.25, 0.50, 0.75, 0.90, 0.95]
        return qs, np.quantile(arr, qs) if arr.size else (qs, np.zeros(len(qs)))

    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    for idx, u in enumerate([1, 2]):
        mask = ut == u
        ax = axs[idx]
        ax.plot(t[mask], nis[mask], label=f"nis update_type {u}", color="C0", linewidth=0.5)
        qs, vals = quantile_lines(nis[mask])
        for q, v in zip(qs, vals):
            ax.axhline(v, linestyle="--", label=f"P{int(q*100)}={v:.2f}")
        ax.set_ylabel("nis")
        ax.legend(fontsize="small", loc="upper right")
        ax.grid(True)

    ax = axs[2]
    ax.plot(t, innov_norm, label="innov_norm", color="C1", linewidth=0.5)
    qs, vals = quantile_lines(innov_norm)
    for q, v in zip(qs, vals):
        ax.axhline(v, linestyle="--", label=f"P{int(q*100)}={v:.2f}")
    ax.set_ylabel("innov_norm")
    ax.set_xlabel("time (s)")
    ax.legend(fontsize="small", loc="upper right")
    ax.grid(True)

    fig.tight_layout()
    fname = os.path.join(out_dir, "time_series_metrics.png")
    fig.savefig(fname)
    print(f"Saved time-series plot: {fname}")
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
