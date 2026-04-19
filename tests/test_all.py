"""
Tests
=====
Unit tests covering:
  - Lane model behaviour
  - Path planning (Dijkstra)
  - Robot state machine
  - Deadlock detection
  - Traffic controller metrics

Run with:
    python -m pytest tests/ -v
  or:
    python tests/test_all.py
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import unittest

from core.lane_graph import LaneGraph, Lane, LaneType, SafetyLevel
from core.robot import Robot, RobotState
from core.traffic_controller import TrafficController, DeadlockDetector
from core.map_builder import build_warehouse_map, get_robot_missions


# ─────────────────────────────────────────────
# Helper: build a tiny 3-node graph for tests
# ─────────────────────────────────────────────

def simple_graph() -> LaneGraph:
    g = LaneGraph()
    g.add_lane(Lane("AB", "A", "B", max_speed=2.0, length=10.0,
                    safety_level=SafetyLevel.LOW, lane_type=LaneType.NORMAL))
    g.add_lane(Lane("BC", "B", "C", max_speed=2.0, length=10.0,
                    safety_level=SafetyLevel.LOW, lane_type=LaneType.NORMAL))
    g.add_lane(Lane("AC", "A", "C", max_speed=1.0, length=25.0,
                    safety_level=SafetyLevel.LOW, lane_type=LaneType.NORMAL))
    return g


# ─────────────────────────────────────────────
# Lane tests
# ─────────────────────────────────────────────

class TestLane(unittest.TestCase):

    def test_robot_enters_exits(self):
        lane = Lane("L1", "X", "Y", 2.0, SafetyLevel.LOW, LaneType.NORMAL, 10.0)
        lane.robot_enters("R1")
        self.assertIn("R1", lane.current_occupants)
        self.assertEqual(lane.historical_usage, 1)
        lane.robot_exits("R1")
        self.assertNotIn("R1", lane.current_occupants)

    def test_congestion_updates(self):
        lane = Lane("L2", "X", "Y", 2.0, SafetyLevel.LOW, LaneType.NORMAL, 10.0)
        lane.robot_enters("R1")
        lane.robot_enters("R2")
        lane.robot_enters("R3")
        # 3/3 robots → congestion = 1.0
        self.assertAlmostEqual(lane.congestion_score, 1.0)

    def test_effective_speed_reduces_with_safety(self):
        lane = Lane("L3", "X", "Y", 2.0, SafetyLevel.HIGH, LaneType.NORMAL, 10.0)
        # HIGH safety → 50% of max
        self.assertAlmostEqual(lane.effective_speed_limit(), 1.0)

    def test_effective_speed_reduces_with_congestion(self):
        lane = Lane("L4", "X", "Y", 2.0, SafetyLevel.LOW, LaneType.NORMAL, 10.0)
        lane.congestion_score = 1.0
        # Full congestion → 30% of max
        self.assertAlmostEqual(lane.effective_speed_limit(), 0.6)

    def test_reservation(self):
        lane = Lane("L5", "X", "Y", 2.0, SafetyLevel.MEDIUM,
                    LaneType.INTERSECTION, 10.0, requires_reservation=True)
        lane.reservation = "R1"
        self.assertTrue(lane.is_reserved_by_other("R2"))
        self.assertFalse(lane.is_reserved_by_other("R1"))

    def test_travel_time(self):
        lane = Lane("L6", "X", "Y", 2.0, SafetyLevel.LOW, LaneType.NORMAL, 10.0)
        # 10m / 2m/s = 5s
        self.assertAlmostEqual(lane.travel_time(), 5.0)


# ─────────────────────────────────────────────
# Graph & path planning tests
# ─────────────────────────────────────────────

class TestLaneGraph(unittest.TestCase):

    def test_path_found(self):
        g = simple_graph()
        path = g.find_path("A", "C")
        self.assertIsNotNone(path)

    def test_direct_path_preferred(self):
        # AB + BC total travel time = 5+5 = 10s
        # AC direct = 25s (much slower speed limit)
        # Dijkstra should prefer AB→BC
        g = simple_graph()
        path = g.find_path("A", "C")
        self.assertEqual(path, ["AB", "BC"])

    def test_no_path_when_all_blocked(self):
        g = simple_graph()
        for lane in g.lanes.values():
            lane.is_blocked = True
        path = g.find_path("A", "C")
        self.assertIsNone(path)

    def test_same_start_and_goal(self):
        g = simple_graph()
        path = g.find_path("A", "A")
        self.assertEqual(path, [])

    def test_undirected_lane_creates_reverse(self):
        g = LaneGraph()
        g.add_lane(Lane("XY", "X", "Y", 2.0, SafetyLevel.LOW,
                        LaneType.NORMAL, 10.0, directed=False))
        # Should be able to go Y→X
        path = g.find_path("Y", "X")
        self.assertIsNotNone(path)
        self.assertEqual(path, ["XY_rev"])

    def test_warehouse_map_loads(self):
        g = build_warehouse_map()
        self.assertGreater(len(g.nodes), 5)
        self.assertGreater(len(g.lanes), 10)


# ─────────────────────────────────────────────
# Robot state machine tests
# ─────────────────────────────────────────────

class TestRobot(unittest.TestCase):

    def _make_robot(self, start="A", goal="C"):
        g = simple_graph()
        return Robot("R_test", g, start, goal), g

    def test_initial_state(self):
        r, _ = self._make_robot()
        self.assertEqual(r.state, RobotState.IDLE)
        self.assertEqual(r.current_node, "A")

    def test_plan_path_success(self):
        r, _ = self._make_robot()
        ok = r.plan_path()
        self.assertTrue(ok)
        self.assertEqual(r.state, RobotState.MOVING)
        self.assertGreater(len(r.path), 0)

    def test_plan_path_impossible(self):
        g = simple_graph()
        for lane in g.lanes.values():
            lane.is_blocked = True
        r = Robot("R_fail", g, "A", "C")
        ok = r.plan_path()
        self.assertFalse(ok)

    def test_robot_moves_over_ticks(self):
        r, _ = self._make_robot()
        r.plan_path()
        for _ in range(1000):  # simulate 100 seconds at dt=0.1
            r.tick(0.1)
            if r.state == RobotState.ARRIVED:
                break
        self.assertEqual(r.state, RobotState.ARRIVED)
        self.assertEqual(r.current_node, "C")

    def test_emergency_stop(self):
        r, _ = self._make_robot()
        r.plan_path()
        r.tick(0.1)
        r.emergency_stop("test")
        self.assertEqual(r.state, RobotState.EMERGENCY_STOP)
        self.assertEqual(r.speed, 0.0)
        self.assertEqual(r.emergency_stops, 1)

    def test_resume_after_stop(self):
        r, _ = self._make_robot()
        r.plan_path()
        r.tick(0.1)
        r.emergency_stop("test")
        r.resume_after_stop()
        self.assertNotEqual(r.state, RobotState.EMERGENCY_STOP)

    def test_replan_increments_counter(self):
        r, _ = self._make_robot()
        r.plan_path()
        r.replan("test replan")
        self.assertEqual(r.replans, 1)


# ─────────────────────────────────────────────
# Traffic controller tests
# ─────────────────────────────────────────────

class TestTrafficController(unittest.TestCase):

    def test_controller_runs(self):
        g = build_warehouse_map()
        ctrl = TrafficController(g)
        missions = get_robot_missions()
        for m in missions:
            ctrl.add_robot(Robot(m["id"], g, m["start"], m["goal"]))
        # Short run — just check it doesn't crash
        ctrl.run(duration=5.0, dt=0.1, verbose=False)
        self.assertGreater(ctrl.sim_time, 0)

    def test_metrics_keys(self):
        g = build_warehouse_map()
        ctrl = TrafficController(g)
        ctrl.add_robot(Robot("R01", g, "ENTRY", "EXIT"))
        ctrl.run(duration=10.0, dt=0.1, verbose=False)
        m = ctrl.get_metrics()
        for key in ("sim_time_s", "robots_total", "robots_arrived",
                    "throughput_per_s", "total_wait_time_s"):
            self.assertIn(key, m)

    def test_all_robots_eventually_arrive(self):
        """All 10 robots should arrive within 300 simulated seconds."""
        g = build_warehouse_map()
        ctrl = TrafficController(g)
        for m in get_robot_missions():
            ctrl.add_robot(Robot(m["id"], g, m["start"], m["goal"]))
        ctrl.run(duration=300.0, dt=0.1, verbose=False)
        arrived = sum(1 for r in ctrl.robots if r.state == RobotState.ARRIVED)
        self.assertEqual(arrived, len(ctrl.robots),
                         f"Only {arrived}/{len(ctrl.robots)} robots arrived")


# ─────────────────────────────────────────────
# Deadlock detector test
# ─────────────────────────────────────────────

class TestDeadlockDetector(unittest.TestCase):

    def test_no_deadlock_in_normal_run(self):
        g = build_warehouse_map()
        ctrl = TrafficController(g)
        for m in get_robot_missions():
            ctrl.add_robot(Robot(m["id"], g, m["start"], m["goal"]))
        ctrl.run(duration=50.0, dt=0.1, verbose=False)
        # Deadlock detector should not crash
        det = DeadlockDetector(ctrl.robots, g)
        det.detect_and_resolve()  # Should complete without error


# ─────────────────────────────────────────────
# Run tests
# ─────────────────────────────────────────────

if __name__ == "__main__":
    loader  = unittest.TestLoader()
    suite   = loader.discover(os.path.dirname(__file__), pattern="test_*.py")
    runner  = unittest.TextTestRunner(verbosity=2)
    result  = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
