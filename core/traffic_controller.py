"""
Traffic Controller
==================
Central coordinator for all robots and lanes.

Responsibilities:
  - Run the simulation clock
  - Enforce safe following distances between robots
  - Detect and resolve deadlocks (cyclic waiting)
  - Broadcast lane state changes to robots
  - Collect metrics and heatmap data
"""

import time
import random
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

from core.lane_graph import LaneGraph, Lane
from core.robot import Robot, RobotState


# ─────────────────────────────────────────────
# Heatmap tracker
# ─────────────────────────────────────────────

class LaneHeatmap:
    """
    Tracks per-lane statistics for visualisation and routing.
    Updated every simulation tick.
    """

    def __init__(self, graph: LaneGraph):
        self.graph = graph
        # Usage over time: lane_id → list of (time, occupant_count)
        self.history: dict[str, list[tuple[float, int]]] = defaultdict(list)
        self.peak_occupancy: dict[str, int] = defaultdict(int)
        self.congestion_alerts: dict[str, int] = defaultdict(int)

    def record(self, sim_time: float):
        for lane_id, lane in self.graph.lanes.items():
            count = len(lane.current_occupants)
            self.history[lane_id].append((sim_time, count))
            if count > self.peak_occupancy[lane_id]:
                self.peak_occupancy[lane_id] = count
            if lane.congestion_score >= 0.7:
                self.congestion_alerts[lane_id] += 1

    def hotspots(self, top_n: int = 5) -> list[tuple[str, int]]:
        """Return the N lanes with the most congestion alerts."""
        ranked = sorted(self.congestion_alerts.items(),
                        key=lambda x: x[1], reverse=True)
        return ranked[:top_n]

    def average_congestion(self, lane_id: str) -> float:
        if lane_id not in self.history or not self.history[lane_id]:
            return 0.0
        counts = [c for _, c in self.history[lane_id]]
        return sum(counts) / len(counts)

    def summary(self) -> dict:
        all_lanes = list(self.graph.lanes.keys())
        return {
            "total_lanes_tracked": len(all_lanes),
            "hotspots": self.hotspots(),
            "peak_occupancy": dict(self.peak_occupancy),
        }


# ─────────────────────────────────────────────
# Deadlock detector
# ─────────────────────────────────────────────

class DeadlockDetector:
    """
    Detects cyclic waiting among robots.

    A deadlock occurs when robot A waits for robot B,
    robot B waits for robot C, …, robot N waits for robot A.

    Detection: Build a "wait-for" graph and look for cycles.
    Resolution: Force the lowest-priority robot in the cycle to replan.
    """

    def __init__(self, robots: list[Robot], graph: LaneGraph):
        self.robots  = robots
        self.graph   = graph
        self.detected_count = 0

    def _build_wait_for_graph(self) -> dict[str, list[str]]:
        """
        Map: robot_id → [robot_ids that are blocking it].
        A robot is blocked when it wants a lane that another robot is on.
        """
        wait_for: dict[str, list[str]] = defaultdict(list)

        for robot in self.robots:
            if robot.state != RobotState.WAITING:
                continue
            if not robot.path:
                continue

            wanted_lane_id = robot.path[0]
            lane = self.graph.get_lane(wanted_lane_id)
            if lane is None:
                continue

            # This robot is waiting for all occupants on the wanted lane
            for occupant_id in lane.current_occupants:
                if occupant_id != robot.robot_id:
                    wait_for[robot.robot_id].append(occupant_id)

            # Also check reservation conflict
            if lane.reservation and lane.reservation != robot.robot_id:
                wait_for[robot.robot_id].append(lane.reservation)

        return dict(wait_for)

    def _find_cycles(self, graph: dict[str, list[str]]) -> list[list[str]]:
        """DFS-based cycle detection. Returns list of cycles found."""
        visited = set()
        in_stack = set()
        cycles = []

        def dfs(node, path):
            visited.add(node)
            in_stack.add(node)

            for neighbour in graph.get(node, []):
                if neighbour not in visited:
                    dfs(neighbour, path + [neighbour])
                elif neighbour in in_stack:
                    # Found a cycle — extract it
                    cycle_start = path.index(neighbour)
                    cycles.append(path[cycle_start:])

            in_stack.discard(node)

        for node in list(graph.keys()):
            if node not in visited:
                dfs(node, [node])

        return cycles

    def detect_and_resolve(self):
        """Check for deadlocks and break any found cycles."""
        wait_graph = self._build_wait_for_graph()
        cycles = self._find_cycles(wait_graph)

        for cycle in cycles:
            self.detected_count += 1
            print(f"\n  ⚡ DEADLOCK detected! Cycle: {' → '.join(cycle)}")
            # Resolution: pick the robot with the most replans (least lucky)
            # and force it to replan with a small delay penalty
            worst = max(
                (r for r in self.robots if r.robot_id in cycle),
                key=lambda r: r.replans
            )
            print(f"     Breaking deadlock: forcing Robot {worst.robot_id} to replan")
            # Release its current lane if any
            if worst.current_lane_id:
                lane = self.graph.get_lane(worst.current_lane_id)
                if lane:
                    lane.robot_exits(worst.robot_id)
                worst.current_lane_id = None
            # Back robot up to its current node and replan
            worst.wait_time += 2.0  # Penalise with a wait
            worst.replan("deadlock resolution")


# ─────────────────────────────────────────────
# Traffic Controller
# ─────────────────────────────────────────────

class TrafficController:
    """
    Orchestrates the entire simulation.

    Usage:
        ctrl = TrafficController(graph)
        ctrl.add_robot(Robot(...))
        ctrl.run(duration=120.0, dt=0.1)
        ctrl.print_report()
    """

    # Minimum gap between robots on the same lane (as fraction of lane length)
    SAFE_FOLLOWING_FRACTION = 0.3

    def __init__(self, graph: LaneGraph):
        self.graph   = graph
        self.robots: list[Robot] = []
        self.heatmap = LaneHeatmap(graph)
        self.sim_time = 0.0
        self._deadlock_check_interval = 5.0   # seconds between checks
        self._last_deadlock_check     = 0.0

    def add_robot(self, robot: Robot):
        """Register a robot and wire up its safety callback."""
        robot._is_safe_to_enter = self._is_safe_to_enter
        self.robots.append(robot)

    def _is_safe_to_enter(self, lane: Lane, robot_id: str) -> bool:
        """
        Safety check: is there enough gap between robots on this lane?
        Uses a simple headway model.
        """
        if not lane.current_occupants:
            return True

        # For narrow lanes, zero-tolerance: must be empty
        from core.lane_graph import LaneType
        if lane.lane_type == LaneType.NARROW:
            return len(lane.current_occupants) == 0

        # For other lanes, check how many are already inside
        max_allowed = {
            LaneType.NORMAL:       2,
            LaneType.INTERSECTION: 1,
            LaneType.HUMAN_ZONE:   1,
        }.get(lane.lane_type, 2)

        return len(lane.current_occupants) < max_allowed

    def _tick_all_robots(self, dt: float):
        """Advance every robot by one time step."""
        for robot in self.robots:
            robot.tick(dt)

    def _check_deadlocks(self):
        """Periodically run deadlock detection."""
        if self.sim_time - self._last_deadlock_check >= self._deadlock_check_interval:
            detector = DeadlockDetector(self.robots, self.graph)
            detector.detect_and_resolve()
            self._last_deadlock_check = self.sim_time

    def _all_done(self) -> bool:
        return all(r.state == RobotState.ARRIVED for r in self.robots)

    def run(self, duration: float = 120.0, dt: float = 0.1,
            verbose: bool = True, print_interval: float = 10.0):
        """
        Main simulation loop.

        Args:
            duration:       Maximum simulation time in seconds.
            dt:             Time step size in seconds.
            verbose:        Print periodic status.
            print_interval: How often (seconds) to print status.
        """
        print(f"\n{'═'*60}")
        print(f"  SIMULATION START  |  {len(self.robots)} robots  |  "
              f"duration={duration}s  dt={dt}s")
        print(f"{'═'*60}\n")

        # Initial path planning for all robots
        for robot in self.robots:
            ok = robot.plan_path()
            if not ok:
                print(f"  ⚠  Robot {robot.robot_id}: no initial path found!")

        last_print = -print_interval  # trigger immediate first print

        while self.sim_time <= duration:
            # Tick all robots
            self._tick_all_robots(dt)

            # Record heatmap
            self.heatmap.record(self.sim_time)

            # Periodic deadlock check
            self._check_deadlocks()

            # Print status
            if verbose and (self.sim_time - last_print) >= print_interval:
                self._print_status()
                last_print = self.sim_time

            # Early exit when all robots are done
            if self._all_done():
                print(f"\n  ✓ All robots reached their goals at t={self.sim_time:.1f}s")
                break

            self.sim_time += dt

        if not self._all_done():
            print(f"\n  ⏱ Simulation ended at t={duration}s — "
                  f"{sum(1 for r in self.robots if r.state != RobotState.ARRIVED)} "
                  f"robots still en route.")

    def _print_status(self):
        print(f"\n  ── t={self.sim_time:6.1f}s ─────────────────────────────────")
        for r in self.robots:
            s = r.status()
            print(f"  Robot {s['id']:>3} | {s['state']:14} | "
                  f"node={s['node']:>6} | lane={s['lane']:>12} | "
                  f"spd={s['speed']:4.1f} | "
                  f"prog={s['progress']:5.1f}% | wait={s['wait_s']:5.1f}s")

    # ── Metrics report ────────────────────────────────

    def get_metrics(self) -> dict:
        """Collect aggregate performance metrics."""
        arrived    = [r for r in self.robots if r.state == RobotState.ARRIVED]
        not_arrived = [r for r in self.robots if r.state != RobotState.ARRIVED]

        total_wait     = sum(r.wait_time        for r in self.robots)
        total_dist     = sum(r.total_distance   for r in self.robots)
        total_replans  = sum(r.replans          for r in self.robots)
        total_estops   = sum(r.emergency_stops  for r in self.robots)
        total_lanes    = sum(r.lanes_traversed  for r in self.robots)

        throughput = len(arrived) / self.sim_time if self.sim_time > 0 else 0

        return {
            "sim_time_s":           round(self.sim_time, 2),
            "robots_total":         len(self.robots),
            "robots_arrived":       len(arrived),
            "robots_incomplete":    len(not_arrived),
            "throughput_per_s":     round(throughput, 4),
            "total_wait_time_s":    round(total_wait, 2),
            "avg_wait_per_robot_s": round(total_wait / max(1, len(self.robots)), 2),
            "total_distance_m":     round(total_dist, 2),
            "total_replans":        total_replans,
            "total_emergency_stops":total_estops,
            "total_lanes_traversed":total_lanes,
            "deadlocks_resolved":   DeadlockDetector(self.robots, self.graph).detected_count,
            "heatmap_hotspots":     self.heatmap.hotspots(),
        }

    def print_report(self):
        """Print a human-readable final report."""
        m = self.get_metrics()
        print(f"\n{'═'*60}")
        print("  SIMULATION RESULTS")
        print(f"{'═'*60}")
        print(f"  Duration              : {m['sim_time_s']} s")
        print(f"  Robots total / arrived: {m['robots_total']} / {m['robots_arrived']}")
        print(f"  Throughput            : {m['throughput_per_s']} robots/s")
        print(f"  Total wait time       : {m['total_wait_time_s']} s")
        print(f"  Avg wait per robot    : {m['avg_wait_per_robot_s']} s")
        print(f"  Total distance        : {m['total_distance_m']} m")
        print(f"  Total replans         : {m['total_replans']}")
        print(f"  Emergency stops       : {m['total_emergency_stops']}")
        print(f"  Lanes traversed       : {m['total_lanes_traversed']}")
        print(f"\n  Lane Heatmap — Top Congestion Hotspots:")
        for lane_id, alerts in m['heatmap_hotspots']:
            avg = round(self.heatmap.average_congestion(lane_id), 2)
            print(f"    {lane_id:20} | congestion alerts={alerts:4d} | avg occupancy={avg}")
        print(f"{'═'*60}\n")

        print("  Per-Robot Summary:")
        print(f"  {'ID':>4} {'State':>14} {'Dist(m)':>8} {'Wait(s)':>8} "
              f"{'Replans':>7} {'EStops':>6} {'Lanes':>6}")
        print(f"  {'-'*58}")
        for r in sorted(self.robots, key=lambda x: x.robot_id):
            s = r.status()
            print(f"  {s['id']:>4} {s['state']:>14} {s['dist_m']:>8} "
                  f"{s['wait_s']:>8} {s['replans']:>7} {s['estops']:>6} "
                  f"{s['lanes_done']:>6}")
        print(f"{'═'*60}\n")
