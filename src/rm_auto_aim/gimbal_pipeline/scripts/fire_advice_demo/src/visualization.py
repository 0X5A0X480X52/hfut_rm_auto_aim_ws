from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Rectangle


def plot_results(rows: list[dict], output_path: str | Path, show: bool = False) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    t = [r["t"] for r in rows]
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, [r["p_hit_window"] for r in rows], label="P_window")
    axes[0].plot(t, [r["fire_score"] for r in rows], label="fire_score")
    axes[0].step(t, [r["fire_advice"] for r in rows], where="post", label="fire_advice")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, [r["e_u"] * 1000.0 for r in rows], label="e_u mm")
    axes[1].plot(t, [r["e_v"] * 1000.0 for r in rows], label="e_v mm")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, [r["sigma_u"] * 1000.0 for r in rows], label="sigma_u mm")
    axes[2].plot(t, [r["sigma_v"] * 1000.0 for r in rows], label="sigma_v mm")
    axes[2].plot(t, [r["best_tau_ms"] for r in rows], label="best tau ms", alpha=0.75)
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, [r["elapsed_us"] for r in rows], label="elapsed us")
    axes[3].plot(t, [r["flight_time_ms"] for r in rows], label="flight time ms")
    axes[3].set_xlabel("time s")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)
    return output_path


def plot_armor_plane_95(rows: list[dict], output_path: str | Path, show: bool = False) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    best = max(rows, key=lambda r: r["p_hit_window"])

    e_u = float(best["e_u"])
    e_v = float(best["e_v"])
    sigma_u = max(float(best["sigma_u"]), 1e-9)
    sigma_v = max(float(best["sigma_v"]), 1e-9)
    width = float(best["armor_width"])
    height = float(best["armor_height"])

    # 95% confidence threshold for 2D Gaussian: chi2(df=2, p=0.95) ~= 5.991
    k95 = np.sqrt(5.991)
    ellipse_w = 2.0 * k95 * sigma_u
    ellipse_h = 2.0 * k95 * sigma_v

    fig, ax = plt.subplots(1, 1, figsize=(6.8, 6.2))
    rect = Rectangle(
        (-width * 0.5 * 1000.0, -height * 0.5 * 1000.0),
        width * 1000.0,
        height * 1000.0,
        edgecolor="tab:green",
        facecolor="none",
        linewidth=2.0,
        label="armor plane",
    )
    ax.add_patch(rect)

    ell = Ellipse(
        (e_u * 1000.0, e_v * 1000.0),
        ellipse_w * 1000.0,
        ellipse_h * 1000.0,
        angle=0.0,
        edgecolor="tab:red",
        facecolor="none",
        linewidth=2.0,
        label="95% confidence ellipse",
    )
    ax.add_patch(ell)
    ax.scatter([e_u * 1000.0], [e_v * 1000.0], c="tab:red", s=40, label="mean impact error")
    ax.scatter([0.0], [0.0], c="tab:blue", s=35, label="armor center")

    lim = max(
        width * 0.7,
        height * 0.7,
        abs(e_u) + ellipse_w * 0.6,
        abs(e_v) + ellipse_h * 0.6,
    ) * 1000.0
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("u (mm)")
    ax.set_ylabel("v (mm)")
    ax.set_title(f"Armor Plane Projection @ max P_hit={best['p_hit_window']:.3f}, t={best['t']:.3f}s")
    ax.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    if show:
        plt.show()
    plt.close(fig)
    return output_path


def plot_projection_trajectory(projection: dict, output_path: str | Path, show: bool = False) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    mean_uv = projection["mean_uv"]
    sigma_uv = projection["sigma_uv"]
    samples = projection["samples_uv"]
    width = float(projection["armor_width"])
    height = float(projection["armor_height"])

    fig, ax = plt.subplots(1, 1, figsize=(7.2, 6.4))
    rect = Rectangle(
        (-width * 0.5 * 1000.0, -height * 0.5 * 1000.0),
        width * 1000.0,
        height * 1000.0,
        edgecolor="tab:green",
        facecolor="none",
        linewidth=2.0,
        label="armor plane",
    )
    ax.add_patch(rect)

    stride = max(1, samples.shape[1] // 9)
    for i in range(0, samples.shape[1], stride):
        ax.scatter(
            samples[:, i, 0] * 1000.0,
            samples[:, i, 1] * 1000.0,
            s=7,
            alpha=0.10,
            color="tab:orange",
            linewidths=0,
        )

    ax.plot(mean_uv[:, 0] * 1000.0, mean_uv[:, 1] * 1000.0, color="tab:red", linewidth=2.0, label="mean trajectory")
    ax.scatter([mean_uv[0, 0] * 1000.0], [mean_uv[0, 1] * 1000.0], c="tab:blue", s=35, label="t=0")
    ax.scatter([mean_uv[-1, 0] * 1000.0], [mean_uv[-1, 1] * 1000.0], c="black", s=40, label="impact")

    upper_u = (mean_uv[:, 0] + 2.0 * sigma_uv[:, 0]) * 1000.0
    lower_u = (mean_uv[:, 0] - 2.0 * sigma_uv[:, 0]) * 1000.0
    upper_v = (mean_uv[:, 1] + 2.0 * sigma_uv[:, 1]) * 1000.0
    lower_v = (mean_uv[:, 1] - 2.0 * sigma_uv[:, 1]) * 1000.0
    ax.plot(upper_u, mean_uv[:, 1] * 1000.0, color="tab:red", alpha=0.35, linewidth=1.0)
    ax.plot(lower_u, mean_uv[:, 1] * 1000.0, color="tab:red", alpha=0.35, linewidth=1.0)
    ax.plot(mean_uv[:, 0] * 1000.0, upper_v, color="tab:purple", alpha=0.35, linewidth=1.0)
    ax.plot(mean_uv[:, 0] * 1000.0, lower_v, color="tab:purple", alpha=0.35, linewidth=1.0)

    lim = np.max(np.abs(np.concatenate([samples[:, :, 0], samples[:, :, 1]], axis=1))) * 1000.0
    lim = max(lim, width * 650.0, height * 650.0)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("u (mm)")
    ax.set_ylabel("v (mm)")
    ax.set_title(
        f"Projected Ballistic Dispersion (t=0 -> impact, tf={projection['flight_time']*1000.0:.1f} ms)"
    )
    ax.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    if show:
        plt.show()
    plt.close(fig)
    return output_path
