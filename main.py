"""
Main Entry Point
================
Runs the Lane-Aware Multi-Robot Traffic Control simulation.

Usage:
    python main.py                    # Default settings
    python main.py --duration 200     # Custom duration
    python main.py --no-charts        # Skip visualisations
    python main.py --dt 0.05          # Finer time step
"""

import argparse
import sys
import os

# Ensure project root is on path when running from any directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.lane_graph   import LaneGraph
from core.map_builder  import build_warehouse_map, get_robot_missions
from core.robot        import Robot
from core.traffic_controller import TrafficController


def parse_args():
    parser = argparse.ArgumentParser(
        description="Lane-Aware Multi-Robot Traffic Control Simulator"
    )
    parser.add_argument("--duration",   type=float, default=180.0,
                        help="Simulation duration in seconds (default: 180)")
    parser.add_argument("--dt",         type=float, default=0.1,
                        help="Time step size in seconds (default: 0.1)")
    parser.add_argument("--interval",   type=float, default=20.0,
                        help="Status print interval in seconds (default: 20)")
    parser.add_argument("--no-charts",  action="store_true",
                        help="Skip generating visualisation charts")
    parser.add_argument("--quiet",      action="store_true",
                        help="Suppress per-step status output")
    return parser.parse_args()


def main():
    args = parse_args()

    print("╔══════════════════════════════════════════════════════════╗")
    print("║   Lane-Aware Multi-Robot Traffic Control System          ║")
    print("║   Hackathon Solution                                     ║")
    print("╚══════════════════════════════════════════════════════════╝")

    # ── 1. Build the warehouse map ─────────────────────────────────────────
    print("\n▶ Building warehouse map …")
    graph = build_warehouse_map()
    print(f"  {graph}")
    print(f"  Lane types present: "
          + ", ".join(sorted(set(l.lane_type.value for l in graph.lanes.values()))))

    # ── 2. Create the traffic controller ──────────────────────────────────
    controller = TrafficController(graph)

    # ── 3. Instantiate robots and assign missions ──────────────────────────
    print("\n▶ Assigning robot missions …")
    missions = get_robot_missions()
    for m in missions:
        robot = Robot(
            robot_id   = m["id"],
            graph      = graph,
            start_node = m["start"],
            goal_node  = m["goal"],
        )
        controller.add_robot(robot)
        print(f"  {m['id']}: {m['start']} → {m['goal']}")

    # ── 4. Run the simulation ──────────────────────────────────────────────
    controller.run(
        duration       = args.duration,
        dt             = args.dt,
        verbose        = not args.quiet,
        print_interval = args.interval,
    )

    # ── 5. Print final report ──────────────────────────────────────────────
    controller.print_report()

    # ── 6. Generate charts ────────────────────────────────────────────────
    if not args.no_charts:
        try:
            from visualization.charts import generate_all_charts
            generate_all_charts(controller)
        except ImportError as e:
            print(f"  ⚠  Could not generate charts (matplotlib missing?): {e}")

    print("\n✓ Simulation complete. Check the results/ folder for charts.\n")


if __name__ == "__main__":
    main()
