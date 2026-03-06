#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_prediction.py
=====================
离线分析 gimbal_pipeline 输出的预测日志 CSV，衡量未来时间窗口内的预测误差。

使用方法：
    python3 analyze_prediction.py --log_dir /tmp/prediction_logs \\
        --r1 0.15 --r2 0.20 --dza 0.0 --max_horizon 0.5

核心思想
--------
对每条装甲板观测 O(T_obs)，在其之前找最近的 tracker 后验状态 S(T_state)：
  dt = T_obs - T_state          ← "预测时长"
利用运动学将 S 传播 dt 秒：
  pred_center(t) = center + vel * dt
  pred_yaw(t)    = yaw + yaw_vel * dt + 0.5 * yaw_acc * dt²
然后计算三类误差：
  ① 机器人中心误差   — 传播中心 vs 观测反推中心
  ② 所有装甲板误差   — 传播4块装甲板位置 vs 观测位置（取最近装甲板）
  ③ 可视装甲板误差   — 同②，但仅统计 visible_armor_count > 0 的时段

可配置的真实结构参数
-------------------
  r1, r2   : 机器人中心到装甲板的旋转半径（默认 0.15, 0.20 m）
  dza      : 奇偶装甲板高度差（默认 0.0 m）
  （CSV 中也记录了 tracker 自估值，可用 --use_estimated_radii 启用）
"""

import os
import sys
import glob
import argparse
import math
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

try:
    from tabulate import tabulate
    HAS_TABULATE = True
except ImportError:
    HAS_TABULATE = False
    warnings.warn("tabulate not installed, table output will be simplified.")

# ─────────────────────────────────────────────────────────────────────────────
# 1.  CLI 参数
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="分析 prediction_logger 输出的 CSV，衡量未来时间窗口预测误差"
    )
    p.add_argument("--log_dir",  default="/tmp/prediction_logs",
                   help="包含 observation_log_*.csv 和 tracker_state_log_*.csv 的目录")
    # 真实结构参数（Python 脚本侧可配置）
    p.add_argument("--r1",   type=float, default=0.15,
                   help="装甲板旋转半径 r1（偶数编号装甲板，默认 0.15 m）")
    p.add_argument("--r2",   type=float, default=0.20,
                   help="装甲板旋转半径 r2（奇数编号装甲板，默认 0.20 m）")
    p.add_argument("--dza",  type=float, default=0.0,
                   help="奇数装甲板高度偏移 dza（默认 0.0 m）")
    p.add_argument("--use_estimated_radii", action="store_true",
                   help="使用 tracker 在 CSV 中估计的 radius_1/2/dza，而非固定值")
    # 预测时长分析参数
    p.add_argument("--max_horizon", type=float, default=0.5,
                   help="分析的最大预测时长（秒，默认 0.5）")
    p.add_argument("--horizon_bins", type=int, default=10,
                   help="预测时长分 N 个等宽区间（默认 10）")
    # 时间分段
    p.add_argument("--time_window", type=float, default=1.0,
                   help="时间窗口分段宽度（秒，默认 1.0）")
    p.add_argument("--session_gap", type=float, default=0.5,
                   help="观测间隔超过此值视为新 session（秒，默认 0.5）")
    # 过滤
    p.add_argument("--robot_id",  default="",
                   help="只分析指定 robot_id；为空则分析全部")
    p.add_argument("--track_state_filter", type=int, default=-1,
                   help="只分析指定 track_state（0=DETECTING,1=TRACKING,2=TEMP_LOST,-1=全部）")
    # 输出
    p.add_argument("--output_dir", default="",
                   help="图表和误差 CSV 的输出目录（默认与 log_dir 相同）")
    p.add_argument("--no_plot", action="store_true", help="不显示交互图窗口")
    p.add_argument("--dpi", type=int, default=120, help="图表 DPI（默认 120）")
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# 2.  数据加载
# ─────────────────────────────────────────────────────────────────────────────

def find_latest_pair(log_dir: str):
    """在目录中找时间戳最新的一对 (obs_csv, state_csv)"""
    obs_files   = sorted(glob.glob(os.path.join(log_dir, "observation_log_*.csv")))
    state_files = sorted(glob.glob(os.path.join(log_dir, "tracker_state_log_*.csv")))
    if not obs_files or not state_files:
        raise FileNotFoundError(
            f"在 {log_dir} 中找不到日志文件。\n"
            "请确认 logging.enable=true 且节点已运行过。"
        )
    return obs_files[-1], state_files[-1]


def load_data(log_dir: str, robot_id_filter: str = ""):
    obs_path, state_path = find_latest_pair(log_dir)
    print(f"[+] 观测日志  : {obs_path}")
    print(f"[+] 状态日志  : {state_path}")

    obs_df   = pd.read_csv(obs_path)
    state_df = pd.read_csv(state_path)

    # 时间戳转秒（float64）
    obs_df["t"]   = obs_df["timestamp_ns"] / 1e9
    state_df["t"] = state_df["timestamp_ns"] / 1e9

    # 过滤 robot_id
    if robot_id_filter:
        obs_df   = obs_df[obs_df["robot_id"] == robot_id_filter].copy()
        state_df = state_df[state_df["robot_id"] == robot_id_filter].copy()

    obs_df.reset_index(drop=True, inplace=True)
    state_df.reset_index(drop=True, inplace=True)

    print(f"[+] 观测条数  : {len(obs_df)}")
    print(f"[+] 状态条数  : {len(state_df)}")
    return obs_df, state_df


# ─────────────────────────────────────────────────────────────────────────────
# 3.  运动学传播
# ─────────────────────────────────────────────────────────────────────────────

def propagate_state(row: pd.Series, dt: float):
    """
    从 tracker 后验状态传播 dt 秒，返回：
      pred_cx, pred_cy, pred_cz          — 预测机器人中心（odom）
      pred_yaw                            — 预测 yaw
      pred_armors: list of (ax, ay, az)  — 4 块装甲板预测位置
    使用固定 r1, r2, dza（或 row 中的估计值，由上层控制）
    """
    pred_cx  = row["center_x"] + row["vel_x"] * dt
    pred_cy  = row["center_y"] + row["vel_y"] * dt
    pred_cz  = row["center_z"] + row["vel_z"] * dt
    pred_yaw = (row["yaw"]
                + row["yaw_velocity"] * dt
                + 0.5 * row["yaw_acceleration"] * dt * dt)
    return pred_cx, pred_cy, pred_cz, pred_yaw


def predict_armors(pred_cx, pred_cy, pred_cz, pred_yaw, r1, r2, dza, n_armors=4):
    """返回 n_armors 块装甲板的预测位置列表，半径按 r1/r2 交替，高度按 dza 交替。

    与 C++ 观测模型一致：
      偶数面板 (0,2): r1, lower, z = center_z - d_za
      奇数面板 (1,3): r2, upper, z = center_z + d_za
    """
    armors = []
    step = 2.0 * math.pi / n_armors
    for i in range(n_armors):
        a_yaw = pred_yaw + i * step
        r = r1 if (i % 2 == 0) else r2
        dz = -dza if (i % 2 == 0) else dza  # lower=-dza, upper=+dza
        ax = pred_cx + r * math.cos(a_yaw)
        ay = pred_cy + r * math.sin(a_yaw)
        az = pred_cz + dz
        armors.append((ax, ay, az))
    return armors


def infer_panel_radius(obs_yaw: float, pred_yaw: float, r1: float, r2: float,
                        dza: float, n_armors: int = 4):
    """根据观测 yaw 与预测 robot_yaw 推断哪块装甲板，返回 (r, dz)。

    obs_yaw 是装甲板法线方向 = robot_yaw + panel_idx * (2π/n)。
    通过取最近的 panel_idx 来决定使用 r1/r2 和 dz。
    """
    step = 2.0 * math.pi / n_armors
    offset = (obs_yaw - pred_yaw) % (2.0 * math.pi)
    panel_idx = int(round(offset / step)) % n_armors
    r  = r1 if (panel_idx % 2 == 0) else r2
    dz = -dza if (panel_idx % 2 == 0) else dza  # lower=-dza, upper=+dza
    return r, dz


# ─────────────────────────────────────────────────────────────────────────────
# 4.  误差计算主循环
# ─────────────────────────────────────────────────────────────────────────────

def compute_errors(obs_df: pd.DataFrame,
                   state_df: pd.DataFrame,
                   args) -> pd.DataFrame:
    """
    对每条观测找最近的前序状态，传播到观测时刻，计算三类误差。
    返回 DataFrame：每行对应一条观测，附加误差列。
    """
    if state_df.empty or obs_df.empty:
        print("[!] 空数据，无法计算误差")
        return pd.DataFrame()

    robot_ids = obs_df["robot_id"].unique() if not args.robot_id else [args.robot_id]

    records = []

    for rid in robot_ids:
        obs_r   = obs_df[obs_df["robot_id"] == rid].sort_values("t").reset_index(drop=True)
        state_r = state_df[state_df["robot_id"] == rid].sort_values("t").reset_index(drop=True)
        if obs_r.empty or state_r.empty:
            continue

        # track_state 过滤
        if args.track_state_filter >= 0:
            state_r = state_r[state_r["track_state"] == args.track_state_filter].reset_index(drop=True)
        if state_r.empty:
            continue

        state_times = state_r["t"].values  # sorted ascending

        for _, obs_row in obs_r.iterrows():
            t_obs = obs_row["t"]

            # 找最近的 **严格前序** 状态（t_state < t_obs）
            # 因为同一回调中 obs 和 state 共享 timestamp，
            # 用 side="left" 确保跳过同帧状态（后验已含本帧观测，不能自比）。
            idx = np.searchsorted(state_times, t_obs, side="left") - 1
            if idx < 0:
                continue   # 没有前序状态

            st = state_r.iloc[idx]
            dt = t_obs - st["t"]

            # 超出分析窗口则跳过
            if dt > args.max_horizon or dt < 0:
                continue

            # 选择结构参数（固定值 or 估计值）
            r1  = st["radius_1"] if args.use_estimated_radii else args.r1
            r2  = st["radius_2"] if args.use_estimated_radii else args.r2
            dza = st["dza"]      if args.use_estimated_radii else args.dza

            # 运动学传播
            pred_cx, pred_cy, pred_cz, pred_yaw = propagate_state(st, dt)

            # ── ① 机器人中心误差 ────────────────────────────────────────────
            # obs_yaw 是装甲板法线方向 = robot_yaw + panel_idx * π/2
            # 根据预测 yaw 推断 panel，选择对应 r 和 dz 来反推真实中心
            obs_yaw  = obs_row["obs_yaw"]
            n_a = int(st["num_armors"]) if st["num_armors"] > 0 else 4
            r_infer, dz_infer = infer_panel_radius(obs_yaw, pred_yaw,
                                                    r1, r2, dza, n_a)
            true_cx  = obs_row["obs_x"] - r_infer * math.cos(obs_yaw)
            true_cy  = obs_row["obs_y"] - r_infer * math.sin(obs_yaw)
            true_cz  = obs_row["obs_z"] - dz_infer

            err_center_2d = math.hypot(pred_cx - true_cx, pred_cy - true_cy)
            err_center_3d = math.sqrt((pred_cx - true_cx)**2 +
                                      (pred_cy - true_cy)**2 +
                                      (pred_cz - true_cz)**2)

            # ── ② 所有装甲板误差（最近装甲板）──────────────────────────────
            pred_armors = predict_armors(pred_cx, pred_cy, pred_cz, pred_yaw,
                                         r1, r2, dza, n_armors=n_a)
            ox, oy, oz = obs_row["obs_x"], obs_row["obs_y"], obs_row["obs_z"]
            err_all_armors_2d = min(
                math.hypot(ax - ox, ay - oy) for ax, ay, _ in pred_armors
            )
            err_all_armors_3d = min(
                math.sqrt((ax-ox)**2 + (ay-oy)**2 + (az-oz)**2)
                for ax, ay, az in pred_armors
            )

            # ── ③ 可视装甲板误差（仅 visible_armor_count > 0 的时段）────────
            is_visible_slot = (st["visible_armor_count"] > 0)
            err_vis_2d = err_all_armors_2d if is_visible_slot else float("nan")
            err_vis_3d = err_all_armors_3d if is_visible_slot else float("nan")

            # ── yaw 误差 ────────────────────────────────────────────────────
            # obs_yaw 是装甲板朝向 = robot_yaw + k*(2π/n_armors)
            # 真实 robot_yaw 对称等价模 (2π/n_armors)，取最小残差
            panel_step = 2.0 * math.pi / n_a
            yaw_diff = (pred_yaw - obs_yaw) % panel_step
            err_yaw  = min(yaw_diff, panel_step - yaw_diff)

            records.append({
                "robot_id"          : rid,
                "t_obs"             : t_obs,
                "t_state"           : st["t"],
                "dt"                : dt,
                "track_state"       : int(st["track_state"]),
                "visible_slot"      : int(is_visible_slot),
                "err_center_2d"     : err_center_2d,
                "err_center_3d"     : err_center_3d,
                "err_all_armors_2d" : err_all_armors_2d,
                "err_all_armors_3d" : err_all_armors_3d,
                "err_vis_2d"        : err_vis_2d,
                "err_vis_3d"        : err_vis_3d,
                "err_yaw_rad"       : err_yaw,
                "err_yaw_deg"       : math.degrees(err_yaw),
            })

    return pd.DataFrame(records)


# ─────────────────────────────────────────────────────────────────────────────
# 5.  统计汇总
# ─────────────────────────────────────────────────────────────────────────────

TRACK_STATE_NAME = {0: "DETECTING", 1: "TRACKING", 2: "TEMP_LOST"}

_METRICS = [
    ("err_center_2d",     "中心2D误差(m)"),
    ("err_center_3d",     "中心3D误差(m)"),
    ("err_all_armors_2d", "全装甲板2D误差(m)"),
    ("err_all_armors_3d", "全装甲板3D误差(m)"),
    ("err_vis_2d",        "可视装甲板2D误差(m)"),
    ("err_yaw_deg",       "Yaw误差(°)"),
]

_LEVELS = [
    ("优秀 (<1 cm)",  0.01),
    ("良好 (<3 cm)",  0.03),
    ("一般 (<5 cm)",  0.05),
    ("较差 (>=5 cm)", float("inf")),
]


def error_level_ratio(series: pd.Series):
    """返回四档误差占比字典（跳过 NaN）"""
    s = series.dropna()
    n = len(s)
    if n == 0:
        return {k: float("nan") for k, _ in _LEVELS}
    ratios = {}
    prev = 0.0
    for label, bound in _LEVELS:
        if math.isinf(bound):
            cnt = (s >= prev).sum()
        else:
            cnt = ((s >= prev) & (s < bound)).sum()
        ratios[label] = cnt / n * 100
        prev = bound
    return ratios


def summarize_by_horizon(df: pd.DataFrame, args):
    """按预测时长分区间汇总三类误差"""
    bins   = np.linspace(0, args.max_horizon, args.horizon_bins + 1)
    labels = [f"{bins[i]*1000:.0f}-{bins[i+1]*1000:.0f}ms"
              for i in range(len(bins) - 1)]
    df = df.copy()
    df["horizon_bin"] = pd.cut(df["dt"], bins=bins, labels=labels, right=True)

    rows = []
    grouped = df.groupby("horizon_bin", observed=False)
    for label, grp in grouped:
        n = len(grp)
        row = {"预测时长": label, "样本数": n}
        for col, _ in _METRICS:
            s = grp[col].dropna()
            if s.empty:
                row[f"{col}_mean"] = float("nan")
                row[f"{col}_rmse"] = float("nan")
                row[f"{col}_p90"]  = float("nan")
            else:
                row[f"{col}_mean"] = s.mean()
                row[f"{col}_rmse"] = math.sqrt((s**2).mean())
                row[f"{col}_p90"]  = s.quantile(0.90)
        rows.append(row)
    return pd.DataFrame(rows)


def summarize_overall(df: pd.DataFrame):
    """全量统计"""
    results = {}
    for col, name in _METRICS:
        s = df[col].dropna()
        if s.empty:
            results[name] = {"N": 0, "Mean": "—", "RMSE": "—",
                             "P50": "—", "P90": "—", "P95": "—"}
        elif col == "err_yaw_deg":
            # Yaw 误差使用度为单位
            results[name] = {
                "N"   : len(s),
                "Mean": f"{s.mean():.2f}°",
                "RMSE": f"{math.sqrt((s**2).mean()):.2f}°",
                "P50" : f"{s.quantile(0.50):.2f}°",
                "P90" : f"{s.quantile(0.90):.2f}°",
                "P95" : f"{s.quantile(0.95):.2f}°",
            }
        else:
            results[name] = {
                "N"   : len(s),
                "Mean": f"{s.mean()*100:.2f} cm",
                "RMSE": f"{math.sqrt((s**2).mean())*100:.2f} cm",
                "P50" : f"{s.quantile(0.50)*100:.2f} cm",
                "P90" : f"{s.quantile(0.90)*100:.2f} cm",
                "P95" : f"{s.quantile(0.95)*100:.2f} cm",
            }
    return results


def identify_sessions(df: pd.DataFrame, gap_s: float):
    """按时间间隔分割连续追踪 session"""
    if df.empty:
        return df
    df = df.sort_values("t_obs").copy()
    diff = df["t_obs"].diff().fillna(0)
    df["session_id"] = (diff > gap_s).cumsum()
    return df


def summarize_by_session(df: pd.DataFrame):
    """每个 session 的误差统计"""
    rows = []
    for sid, grp in df.groupby("session_id"):
        t_start = grp["t_obs"].min()
        t_end   = grp["t_obs"].max()
        row = {
            "Session": int(sid),
            "起始时间(s)": f"{t_start:.2f}",
            "时长(s)": f"{t_end - t_start:.2f}",
            "样本数": len(grp),
        }
        for col, name in _METRICS[:4]:
            s = grp[col].dropna()
            row[f"{name}_RMSE"] = f"{math.sqrt((s**2).mean())*100:.2f}" if len(s) else "—"
        rows.append(row)
    return pd.DataFrame(rows)


def summarize_by_time_window(df: pd.DataFrame, window_s: float):
    """按固定时间窗口（wall clock）分段汇总"""
    if df.empty:
        return pd.DataFrame()
    t0 = df["t_obs"].min()
    df = df.copy()
    df["tw"] = ((df["t_obs"] - t0) / window_s).astype(int)
    rows = []
    for tw, grp in df.groupby("tw"):
        row = {"时间窗口": f"{tw*window_s:.1f}-{(tw+1)*window_s:.1f}s",
               "样本数": len(grp)}
        for col, name in _METRICS[:4]:
            s = grp[col].dropna()
            row[f"{name}_RMSE"] = math.sqrt((s**2).mean()) if len(s) else float("nan")
        rows.append(row)
    return pd.DataFrame(rows)


def print_table(df_or_dict, title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")
    if isinstance(df_or_dict, dict):
        data = [(k, *v.values()) for k, v in df_or_dict.items()]
        headers = ["指标"] + list(next(iter(df_or_dict.values())).keys())
    else:
        data = df_or_dict.values.tolist()
        headers = df_or_dict.columns.tolist()
    if HAS_TABULATE:
        print(tabulate(data, headers=headers, tablefmt="rounded_outline",
                       floatfmt=".4f"))
    else:
        print("\t".join(str(h) for h in headers))
        for row in data:
            print("\t".join(str(v) for v in row))


# ─────────────────────────────────────────────────────────────────────────────
# 6.  可视化
# ─────────────────────────────────────────────────────────────────────────────

def _cm(x):
    """米 → 厘米，保留 NaN"""
    return x * 100 if not (isinstance(x, float) and math.isnan(x)) else x


def plot_horizon_curves(horizon_df: pd.DataFrame, output_dir: str, dpi: int):
    """图1：各预测时长区间的三类误差（RMSE 曲线）"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("未来时间窗口预测误差（RMSE）随预测时长变化", fontsize=14)

    triples = [
        ("err_center_2d_rmse",     "中心2D误差"),
        ("err_all_armors_2d_rmse", "全装甲板2D误差"),
        ("err_vis_2d_rmse",        "可视装甲板2D误差"),
    ]
    x_labels = horizon_df["预测时长"].astype(str).tolist()
    x = np.arange(len(x_labels))

    for ax, (col, title) in zip(axes, triples):
        means_col = col.replace("rmse", "mean")
        p90_col   = col.replace("rmse", "p90")
        means = horizon_df[means_col].values if means_col in horizon_df.columns else np.full(len(horizon_df), np.nan)
        rmse  = horizon_df[col].values      if col      in horizon_df.columns else np.full(len(horizon_df), np.nan)
        p90   = horizon_df[p90_col].values  if p90_col  in horizon_df.columns else np.full(len(horizon_df), np.nan)

        ax.plot(x, rmse  * 100, "o-",  label="RMSE",  linewidth=2)
        ax.plot(x, means * 100, "s--", label="Mean",  linewidth=1.5, alpha=0.7)
        ax.plot(x, p90   * 100, "^:",  label="P90",   linewidth=1.5, alpha=0.7)

        ax.set_title(title)
        ax.set_xlabel("预测时长")
        ax.set_ylabel("误差 (cm)")
        ax.set_xticks(x)
        ax.set_xticklabels(x_labels, rotation=35, ha="right", fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.1f"))

        # 误差等级参考线（1cm / 3cm / 5cm）
        for level_cm, c in zip([1, 3, 5], ["green", "orange", "red"]):
            ax.axhline(level_cm, color=c, linestyle="--", linewidth=0.8, alpha=0.5,
                       label=f"{level_cm} cm")

    plt.tight_layout()
    out = os.path.join(output_dir, "fig1_horizon_rmse.png")
    plt.savefig(out, dpi=dpi)
    print(f"[+] 保存图1: {out}")
    return fig


def plot_error_cdf(err_df: pd.DataFrame, output_dir: str, dpi: int):
    """图2：三类误差 CDF（累积分布函数）"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("预测误差累积分布（CDF）", fontsize=14)

    pairs = [
        ("err_center_2d",     "中心2D误差 (m)"),
        ("err_all_armors_2d", "全装甲板2D误差 (m)"),
        ("err_vis_2d",        "可视装甲板2D误差 (m)"),
    ]
    for ax, (col, xlabel) in zip(axes, pairs):
        s = err_df[col].dropna().sort_values().values  # numpy array
        if len(s) == 0:
            ax.text(0.5, 0.5, "no data", ha="center", va="center",
                    transform=ax.transAxes)
            ax.set_title(xlabel)
            continue
        cdf = np.arange(1, len(s) + 1) / len(s)
        ax.plot(s * 100, cdf, linewidth=2)
        # 标注 P50 / P90 / P95
        for q, c in [(0.50, "b"), (0.90, "g"), (0.95, "r")]:
            val = np.percentile(s, q * 100) * 100
            ax.axvline(val, color=c, linestyle="--", linewidth=1,
                       label=f"P{int(q*100)}={val:.2f}cm")
        ax.set_title(col.replace("err_", "").replace("_", " "))
        ax.set_xlabel("Error (cm)")
        ax.set_ylabel("CDF")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(output_dir, "fig2_error_cdf.png")
    plt.savefig(out, dpi=dpi)
    print(f"[+] 保存图2: {out}")
    return fig


def plot_time_series(err_df: pd.DataFrame, output_dir: str, dpi: int,
                     rolling_window: int = 30):
    """图3：误差时序（含滚动均值）"""
    if err_df.empty:
        return
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("预测误差时序（含滚动均值）", fontsize=14)

    pairs = [
        ("err_center_2d",     "机器人中心2D误差"),
        ("err_all_armors_2d", "全装甲板2D误差"),
        ("err_vis_2d",        "可视装甲板2D误差"),
    ]
    df_sorted = err_df.sort_values("t_obs").reset_index(drop=True)
    t0 = df_sorted["t_obs"].min()

    for ax, (col, title) in zip(axes, pairs):
        s = (df_sorted[col] * 100).values  # numpy, cm
        t = (df_sorted["t_obs"] - t0).values
        ax.plot(t, s, ".", markersize=2, alpha=0.4, color="steelblue")
        # rolling mean via numpy convolution (avoids pandas Series plotting issues)
        kernel = np.ones(rolling_window) / rolling_window
        valid = ~np.isnan(s)
        roll_mean = np.full_like(s, np.nan)
        if valid.sum() > rolling_window:
            roll_mean[valid] = np.convolve(
                s[valid], kernel, mode="same")
        ax.plot(t, roll_mean, linewidth=2, color="crimson",
                label=f"Rolling mean({rolling_window})")
        ax.set_ylabel("Error (cm)")
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        for level_cm, c in zip([1, 3, 5], ["green", "orange", "red"]):
            ax.axhline(level_cm, linestyle="--", linewidth=0.7,
                       color=c, alpha=0.6)

    axes[-1].set_xlabel("时间 (s，相对起始)")
    plt.tight_layout()
    out = os.path.join(output_dir, "fig3_error_timeseries.png")
    plt.savefig(out, dpi=dpi)
    print(f"[+] 保存图3: {out}")
    return fig


def plot_heatmap(tw_df: pd.DataFrame, output_dir: str, dpi: int):
    """图4：时间窗口误差热图"""
    if tw_df.empty:
        return
    cols_to_plot = [c for c in tw_df.columns
                    if c.endswith("_RMSE") and not tw_df[c].isna().all()]
    if not cols_to_plot:
        return

    data = tw_df[cols_to_plot].values.astype(float) * 100  # cm

    fig, ax = plt.subplots(figsize=(max(8, len(tw_df) * 0.4), 4))
    im = ax.imshow(data.T, aspect="auto", cmap="YlOrRd", vmin=0)
    ax.set_xticks(np.arange(len(tw_df)))
    ax.set_xticklabels(tw_df["时间窗口"].tolist(), rotation=40, ha="right", fontsize=7)
    ax.set_yticks(np.arange(len(cols_to_plot)))
    ax.set_yticklabels([c.replace("_RMSE", "").replace("err_", "") for c in cols_to_plot])
    plt.colorbar(im, ax=ax, label="RMSE (cm)")
    ax.set_title("各时间窗口预测误差热图（RMSE, cm）")
    plt.tight_layout()
    out = os.path.join(output_dir, "fig4_time_window_heatmap.png")
    plt.savefig(out, dpi=dpi)
    print(f"[+] 保存图4: {out}")
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# 7.  主函数
# ─────────────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    output_dir = args.output_dir if args.output_dir else args.log_dir
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("  gimbal_pipeline 预测误差分析")
    print("=" * 60)
    print(f"  r1={args.r1} m  r2={args.r2} m  dza={args.dza} m")
    print(f"  最大预测时长: {args.max_horizon*1000:.0f} ms  |  "
          f"时长分箱: {args.horizon_bins}")
    print(f"  时间窗口: {args.time_window} s  |  "
          f"session 间隔: {args.session_gap} s")
    if args.use_estimated_radii:
        print("  [注] 使用 tracker 估计的结构参数（r1/r2/dza）")

    # 1. 加载数据
    obs_df, state_df = load_data(args.log_dir, args.robot_id)

    # 2. 计算误差
    print("\n[...] 计算预测误差中 ...")
    err_df = compute_errors(obs_df, state_df, args)
    if err_df.empty:
        print("[!] 没有有效误差记录，请检查日志和参数。")
        sys.exit(0)

    print(f"[+] 有效误差样本: {len(err_df)}")

    # 3. 添加 session 标签
    err_df = identify_sessions(err_df, args.session_gap)

    # 4. 汇总
    overall_stats  = summarize_overall(err_df)
    horizon_df     = summarize_by_horizon(err_df, args)
    session_df     = summarize_by_session(err_df)
    tw_df          = summarize_by_time_window(err_df, args.time_window)

    # 5. 打印汇总表
    print_table(overall_stats, "全量误差汇总")
    print_table(horizon_df[["预测时长", "样本数",
                             "err_center_2d_rmse", "err_center_2d_mean",
                             "err_all_armors_2d_rmse", "err_all_armors_2d_mean",
                             "err_vis_2d_rmse"]].rename(columns={
        "err_center_2d_rmse"      : "中心RMSE(m)",
        "err_center_2d_mean"      : "中心Mean(m)",
        "err_all_armors_2d_rmse"  : "全板RMSE(m)",
        "err_all_armors_2d_mean"  : "全板Mean(m)",
        "err_vis_2d_rmse"         : "可视板RMSE(m)",
    }), "按预测时长分区间汇总")
    print_table(session_df, "各 Session 误差汇总")

    # 可视装甲板误差等级分布
    vis_s  = err_df["err_vis_2d"].dropna()
    all_s  = err_df["err_all_armors_2d"].dropna()
    cent_s = err_df["err_center_2d"].dropna()
    print("\n误差等级分布（2D）：")
    for label, s in [("中心误差", cent_s), ("全装甲板", all_s), ("可视装甲板", vis_s)]:
        levels = error_level_ratio(s)
        print(f"  [{label}]  " +
              "  ".join(f"{k}: {v:.1f}%" for k, v in levels.items()))

    # 6. 保存误差 CSV
    out_csv = os.path.join(output_dir, "prediction_errors.csv")
    err_df.to_csv(out_csv, index=False)
    print(f"\n[+] 误差详细 CSV: {out_csv}")

    horizon_csv = os.path.join(output_dir, "horizon_summary.csv")
    horizon_df.to_csv(horizon_csv, index=False)
    print(f"[+] 时长汇总 CSV: {horizon_csv}")

    # 7. 绘图
    print("\n[...] 生成图表 ...")
    plot_horizon_curves(horizon_df, output_dir, args.dpi)
    plot_error_cdf(err_df, output_dir, args.dpi)
    plot_time_series(err_df, output_dir, args.dpi)
    plot_heatmap(tw_df, output_dir, args.dpi)

    if not args.no_plot:
        plt.show()

    print("\n[✓] 分析完成！")


if __name__ == "__main__":
    main()
