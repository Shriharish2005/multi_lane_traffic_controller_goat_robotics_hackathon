"""
Visualizer
==========
Generates a suite of matplotlib charts showing:
  1. Lane heatmap (usage frequency)
  2. Congestion timeline per lane
  3. Per-robot metrics bar charts
  4. Robot state distribution (pie chart)

Saves all charts to results/ directory and shows them.
"""

import os
import math
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend for servers without display
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
import numpy as np

from core.lane_graph import LaneGraph, LaneType, SafetyLevel
from core.traffic_controller import TrafficController, LaneHeatmap
from core.robot import Robot, RobotState


# ─────────────────────────────────────────────
# Node positions for the warehouse layout
# ─────────────────────────────────────────────

NODE_POSITIONS = {
    "ENTRY": (0, 1),
    "N1":    (2, 2),
    "N2":    (4, 2),
    "N3":    (6, 2),
    "N4":    (2, 1),
    "N5":    (4, 1),
    "N6":    (6, 1),
    "N7":    (2, 0),
    "N8":    (4, 0),
    "N9":    (6, 0),
    "N10":   (4, -1),
    "EXIT":  (8, 0),
}

# Color for each lane type
LANE_TYPE_COLORS = {
    LaneType.NORMAL:       "#4A90D9",
    LaneType.NARROW:       "#E67E22",
    LaneType.INTERSECTION: "#9B59B6",
    LaneType.HUMAN_ZONE:   "#E74C3C",
}


def _ensure_results_dir():
    os.makedirs("results", exist_ok=True)


def plot_lane_map(graph: LaneGraph, heatmap: LaneHeatmap, save_path: str = "results/lane_map.png"):
    """
    Draw the warehouse graph with lanes colored by usage heat.
    """
    _ensure_results_dir()
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.set_facecolor("#1a1a2e")
    fig.patch.set_facecolor("#16213e")

    # Custom colormap: blue (cold/unused) → red (hot/heavy usage)
    cmap = LinearSegmentedColormap.from_list(
        "heat", ["#2ecc71", "#f39c12", "#e74c3c"], N=256
    )

    # Find max usage for normalisation
    max_usage = max((l.historical_usage for l in graph.lanes.values()), default=1)
    if max_usage == 0:
        max_usage = 1

    # Draw lanes
    drawn_pairs = set()
    for lane_id, lane in graph.lanes.items():
        fn = lane.from_node
        tn = lane.to_node
        pair = tuple(sorted([fn, tn]))
        if pair in drawn_pairs:
            continue  # Draw undirected lanes only once
        drawn_pairs.add(pair)

        if fn not in NODE_POSITIONS or tn not in NODE_POSITIONS:
            continue

        x0, y0 = NODE_POSITIONS[fn]
        x1, y1 = NODE_POSITIONS[tn]

        heat = lane.historical_usage / max_usage
        color = cmap(heat)
        lw = 2 + heat * 6

        ax.plot([x0, x1], [y0, y1], color=color, linewidth=lw,
                alpha=0.85, solid_capstyle="round")

        # Label the lane
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        ax.text(mx, my + 0.08, lane_id, fontsize=7,
                ha="center", va="bottom", color="white", alpha=0.7)

    # Draw nodes
    for node, (x, y) in NODE_POSITIONS.items():
        is_terminal = node in ("ENTRY", "EXIT")
        color  = "#F1C40F" if is_terminal else "#ECF0F1"
        radius = 0.18    if is_terminal else 0.13
        circle = plt.Circle((x, y), radius, color=color, zorder=5)
        ax.add_patch(circle)
        ax.text(x, y - 0.25, node, fontsize=8, ha="center",
                color=color, fontweight="bold")

    # Colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, max_usage))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("Historical Usage Count", color="white")
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")

    # Legend
    legend_handles = [
        mpatches.Patch(color=c, label=lt.value)
        for lt, c in LANE_TYPE_COLORS.items()
    ]
    ax.legend(handles=legend_handles, loc="upper right",
              facecolor="#0f3460", edgecolor="white",
              labelcolor="white", fontsize=8)

    ax.set_xlim(-0.5, 9)
    ax.set_ylim(-1.8, 2.8)
    ax.set_aspect("equal")
    ax.set_title("Warehouse Lane Map — Usage Heatmap",
                 color="white", fontsize=14, pad=12)
    ax.axis("off")

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Saved: {save_path}")


def plot_heatmap_bars(heatmap: LaneHeatmap, save_path: str = "results/heatmap_bars.png"):
    """Bar chart showing historical usage count per lane."""
    _ensure_results_dir()

    lane_ids = sorted(heatmap.peak_occupancy.keys())
    if not lane_ids:
        print("  No heatmap data to plot.")
        return

    usages  = [heatmap.graph.lanes[lid].historical_usage
                if lid in heatmap.graph.lanes else 0 for lid in lane_ids]
    alerts  = [heatmap.congestion_alerts[lid] for lid in lane_ids]
    peaks   = [heatmap.peak_occupancy[lid]    for lid in lane_ids]

    x = np.arange(len(lane_ids))
    width = 0.28

    fig, ax = plt.subplots(figsize=(16, 6))
    ax.set_facecolor("#1a1a2e")
    fig.patch.set_facecolor("#16213e")

    b1 = ax.bar(x - width, usages,  width, label="Historical Usage",  color="#3498db", alpha=0.85)
    b2 = ax.bar(x,          alerts,  width, label="Congestion Alerts",  color="#e74c3c", alpha=0.85)
    b3 = ax.bar(x + width,  peaks,   width, label="Peak Occupancy",     color="#f39c12", alpha=0.85)

    ax.set_xticks(x)
    ax.set_xticklabels(lane_ids, rotation=45, ha="right", color="white", fontsize=8)
    ax.tick_params(colors="white")
    ax.spines["bottom"].set_color("#555")
    ax.spines["left"].set_color("#555")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.set_ylabel("Count", color="white")
    ax.set_title("Lane Heatmap — Usage, Congestion Alerts & Peak Occupancy",
                 color="white", fontsize=13)
    ax.legend(facecolor="#0f3460", edgecolor="white",
              labelcolor="white", fontsize=9)
    ax.yaxis.label.set_color("white")

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Saved: {save_path}")


def plot_robot_metrics(robots: list[Robot], save_path: str = "results/robot_metrics.png"):
    """Grouped bar chart: wait time, distance, replans per robot."""
    _ensure_results_dir()

    ids      = [r.robot_id for r in robots]
    waits    = [r.wait_time      for r in robots]
    dists    = [r.total_distance for r in robots]
    replans  = [r.replans        for r in robots]

    x     = np.arange(len(ids))
    width = 0.25

    fig, ax = plt.subplots(figsize=(14, 6))
    ax.set_facecolor("#1a1a2e")
    fig.patch.set_facecolor("#16213e")

    ax.bar(x - width, waits,   width, label="Wait Time (s)",    color="#e74c3c", alpha=0.9)
    ax.bar(x,          dists,   width, label="Distance (m)",     color="#2ecc71", alpha=0.9)
    ax.bar(x + width,  replans, width, label="Replans",          color="#f39c12", alpha=0.9)

    ax.set_xticks(x)
    ax.set_xticklabels(ids, color="white", fontsize=9)
    ax.tick_params(colors="white")
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["bottom", "left"]:
        ax.spines[spine].set_color("#555")
    ax.set_ylabel("Value", color="white")
    ax.yaxis.label.set_color("white")
    ax.set_title("Per-Robot Performance Metrics", color="white", fontsize=13)
    ax.legend(facecolor="#0f3460", edgecolor="white",
              labelcolor="white", fontsize=9)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Saved: {save_path}")


def plot_state_distribution(robots: list[Robot],
                             save_path: str = "results/state_distribution.png"):
    """Pie chart showing final state distribution of all robots."""
    _ensure_results_dir()

    state_counts = {}
    for r in robots:
        name = r.state.name
        state_counts[name] = state_counts.get(name, 0) + 1

    labels = list(state_counts.keys())
    sizes  = list(state_counts.values())
    colors_map = {
        "ARRIVED":        "#2ecc71",
        "MOVING":         "#3498db",
        "WAITING":        "#f39c12",
        "EMERGENCY_STOP": "#e74c3c",
        "PLANNING":       "#9b59b6",
        "IDLE":           "#95a5a6",
    }
    colors = [colors_map.get(l, "#bdc3c7") for l in labels]

    fig, ax = plt.subplots(figsize=(7, 7))
    fig.patch.set_facecolor("#16213e")
    ax.set_facecolor("#16213e")

    wedges, texts, autotexts = ax.pie(
        sizes, labels=labels, colors=colors,
        autopct="%1.0f%%", startangle=140,
        textprops={"color": "white", "fontsize": 11},
        wedgeprops={"edgecolor": "#16213e", "linewidth": 2}
    )
    for at in autotexts:
        at.set_color("white")
        at.set_fontsize(10)

    ax.set_title("Final Robot State Distribution",
                 color="white", fontsize=13, pad=16)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Saved: {save_path}")


def plot_congestion_timeline(heatmap: LaneHeatmap,
                              top_n: int = 6,
                              save_path: str = "results/congestion_timeline.png"):
    """Line chart: congestion occupancy over time for the busiest lanes."""
    _ensure_results_dir()

    hotspot_ids = [lid for lid, _ in heatmap.hotspots(top_n=top_n)]
    if not hotspot_ids:
        print("  No hotspot data for timeline.")
        return

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.set_facecolor("#1a1a2e")
    fig.patch.set_facecolor("#16213e")

    palette = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12", "#9b59b6", "#1abc9c"]

    for i, lid in enumerate(hotspot_ids):
        data = heatmap.history.get(lid, [])
        if not data:
            continue
        times  = [t for t, _ in data]
        counts = [c for _, c in data]
        ax.plot(times, counts, label=lid,
                color=palette[i % len(palette)],
                linewidth=1.5, alpha=0.85)

    ax.set_xlabel("Simulation Time (s)", color="white")
    ax.set_ylabel("Occupant Count", color="white")
    ax.set_title(f"Congestion Timeline — Top {top_n} Hotspot Lanes",
                 color="white", fontsize=13)
    ax.tick_params(colors="white")
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["bottom", "left"]:
        ax.spines[spine].set_color("#555")
    ax.legend(facecolor="#0f3460", edgecolor="white",
              labelcolor="white", fontsize=9)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Saved: {save_path}")


def generate_all_charts(controller: TrafficController):
    """Generate and save all visualisation charts."""
    print("\n  Generating visualisations …")
    plot_lane_map(controller.graph, controller.heatmap)
    plot_heatmap_bars(controller.heatmap)
    plot_robot_metrics(controller.robots)
    plot_state_distribution(controller.robots)
    plot_congestion_timeline(controller.heatmap)
    print("  All charts saved to results/")
