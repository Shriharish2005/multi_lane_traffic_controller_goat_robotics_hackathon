"""
Robot Module
============
Each robot is an autonomous agent that:
  - Follows a lane-based path from start → goal
  - Adapts speed to lane conditions
  - Reserves critical lanes before entering
  - Replans dynamically when lanes are blocked or congested
  - Can emergency-stop
"""

import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

from core.lane_graph import LaneGraph, Lane, SafetyLevel, LaneType


# ─────────────────────────────────────────────
# Robot states
# ─────────────────────────────────────────────

class RobotState(Enum):
    IDLE           = auto()   # Waiting for a task
    PLANNING       = auto()   # Computing a path
    MOVING         = auto()   # Traversing a lane
    WAITING        = auto()   # Blocked; waiting for access
    EMERGENCY_STOP = auto()   # Halted due to safety event
    ARRIVED        = auto()   # Reached goal


# ─────────────────────────────────────────────
# Robot class
# ─────────────────────────────────────────────

@dataclass
class Robot:
    """
    Represents a single robot in the warehouse.

    A robot moves one lane at a time. After completing each lane,
    it advances to the next lane in its path. If the next lane is
    blocked, reserved, or too congested, it replans its route.
    """

    robot_id:     str
    graph:        LaneGraph
    start_node:   str
    goal_node:    str

    # Runtime state
    state:            RobotState = RobotState.IDLE
    current_node:     str        = ""
    current_lane_id:  Optional[str] = None   # Lane the robot is currently on
    path:             list       = field(default_factory=list)  # Remaining lane_ids
    speed:            float      = 0.0        # Current speed in m/s
    position_on_lane: float      = 0.0        # 0.0 = entry, 1.0 = exit

    # Metrics
    total_distance:   float      = 0.0
    total_time:       float      = 0.0
    wait_time:        float      = 0.0
    lanes_traversed:  int        = 0
    replans:          int        = 0
    emergency_stops:  int        = 0

    # Internal timer for lane traversal simulation
    _lane_elapsed:    float      = 0.0
    _lane_duration:   float      = 0.0

    # Safe following distance check callback (set by TrafficController)
    _is_safe_to_enter: object    = None  # callable(lane, robot_id) → bool

    def __post_init__(self):
        self.current_node = self.start_node

    # ── Path planning ─────────────────────────────────

    def plan_path(self) -> bool:
        """Ask the graph for an optimal path. Returns True if a path was found."""
        self.state = RobotState.PLANNING
        path = self.graph.find_path(
            self.current_node, self.goal_node, robot_id=self.robot_id
        )
        if path is None:
            # No route at all — stay in WAITING
            self.state = RobotState.WAITING
            return False
        self.path = path
        self.state = RobotState.MOVING if path else RobotState.ARRIVED
        return True

    def replan(self, reason: str = ""):
        """Force a fresh path computation (called when current path is invalid)."""
        print(f"    [Robot {self.robot_id}] Replanning — {reason}")
        self.replans += 1
        # Exit current lane if on one
        if self.current_lane_id:
            lane = self.graph.get_lane(self.current_lane_id)
            if lane:
                lane.robot_exits(self.robot_id)
            self.current_lane_id = None
        self.plan_path()

    # ── Lane reservation ──────────────────────────────

    def try_reserve_lane(self, lane: Lane) -> bool:
        """
        For critical lanes: attempt to reserve before entering.
        Returns True if reservation was granted.
        """
        if not lane.requires_reservation:
            return True  # No reservation needed
        if lane.reservation is None:
            lane.reservation = self.robot_id
            return True
        if lane.reservation == self.robot_id:
            return True  # Already ours
        return False  # Reserved by someone else

    # ── Safety checks ─────────────────────────────────

    def _check_safe_following(self, lane: Lane) -> bool:
        """
        Ensure there is adequate following distance before entering a lane.
        Delegates to TrafficController callback if provided.
        """
        if self._is_safe_to_enter:
            return self._is_safe_to_enter(lane, self.robot_id)
        return True  # Default: allow

    def emergency_stop(self, reason: str = ""):
        """Halt the robot immediately."""
        print(f"    [Robot {self.robot_id}] ⚠ EMERGENCY STOP — {reason}")
        self.emergency_stops += 1
        self.speed = 0.0
        self.state = RobotState.EMERGENCY_STOP
        if self.current_lane_id:
            lane = self.graph.get_lane(self.current_lane_id)
            if lane and self.robot_id in lane.current_occupants:
                lane.robot_exits(self.robot_id)
            self.current_lane_id = None

    def resume_after_stop(self):
        """Resume movement after an emergency stop is cleared."""
        if self.state == RobotState.EMERGENCY_STOP:
            self.state = RobotState.PLANNING
            self.plan_path()

    # ── Main update tick ──────────────────────────────

    def tick(self, dt: float):
        """
        Advance robot state by dt seconds.
        Called once per simulation step.
        """
        if self.state in (RobotState.ARRIVED, RobotState.IDLE,
                           RobotState.EMERGENCY_STOP):
            return

        self.total_time += dt

        # ── Currently traversing a lane ──
        if self.state == RobotState.MOVING and self.current_lane_id:
            self._tick_moving(dt)
            return

        # ── Need to pick the next lane ──
        if self.state in (RobotState.MOVING, RobotState.WAITING, RobotState.PLANNING):
            self._try_enter_next_lane(dt)

    def _tick_moving(self, dt: float):
        """Progress along the current lane."""
        lane = self.graph.get_lane(self.current_lane_id)
        if lane is None:
            self.replan("lane disappeared")
            return

        # Update speed to current lane effective limit
        self.speed = lane.effective_speed_limit()

        self._lane_elapsed += dt
        self.position_on_lane = min(1.0, self._lane_elapsed / self._lane_duration)

        if self._lane_elapsed >= self._lane_duration:
            # Finished traversing this lane
            self._finish_lane(lane)

    def _finish_lane(self, lane: Lane):
        """Called when robot completes a lane traversal."""
        lane.robot_exits(self.robot_id)
        self.current_node = lane.to_node
        self.total_distance += lane.length
        self.lanes_traversed += 1
        self.current_lane_id = None
        self.position_on_lane = 0.0
        self._lane_elapsed = 0.0

        if self.current_node == self.goal_node:
            self.state = RobotState.ARRIVED
            self.speed = 0.0
            print(f"    [Robot {self.robot_id}] ✓ Arrived at goal '{self.goal_node}'")
        else:
            self.state = RobotState.MOVING

    def _try_enter_next_lane(self, dt: float):
        """Attempt to enter the next lane in the path."""
        if not self.path:
            # No more lanes — check if we're at goal
            if self.current_node == self.goal_node:
                self.state = RobotState.ARRIVED
            else:
                self.replan("path exhausted but not at goal")
            return

        next_lane_id = self.path[0]
        lane = self.graph.get_lane(next_lane_id)

        if lane is None:
            self.replan(f"lane {next_lane_id} not found")
            return

        # ── Gate checks before entering ──

        # 1. Lane physically blocked?
        if lane.is_blocked:
            self.wait_time += dt
            self.state = RobotState.WAITING
            # Try to find a new route around it
            self.replan(f"lane {next_lane_id} is blocked")
            return

        # 2. Needs reservation?
        if not self.try_reserve_lane(lane):
            self.wait_time += dt
            self.state = RobotState.WAITING
            return

        # 3. Safe following distance?
        if not self._check_safe_following(lane):
            self.wait_time += dt
            self.state = RobotState.WAITING
            return

        # 4. Narrow lane: only 1 robot allowed
        if lane.lane_type == LaneType.NARROW and len(lane.current_occupants) >= 1:
            self.wait_time += dt
            self.state = RobotState.WAITING
            return

        # 5. Very heavy congestion → replan if possible
        if lane.congestion_score >= 0.9:
            alt_path = self.graph.find_path(
                self.current_node, self.goal_node, robot_id=self.robot_id
            )
            if alt_path and alt_path != self.path:
                print(f"    [Robot {self.robot_id}] Rerouting around congestion on {next_lane_id}")
                self.path = alt_path
                self.replans += 1
                return

        # ── All checks passed: enter the lane ──
        self.path.pop(0)
        lane.robot_enters(self.robot_id)
        self.current_lane_id = next_lane_id
        self.speed = lane.effective_speed_limit()
        self._lane_elapsed = 0.0
        self._lane_duration = lane.travel_time()
        self.position_on_lane = 0.0
        self.state = RobotState.MOVING

    # ── Status summary ────────────────────────────────

    def status(self) -> dict:
        return {
            "id":            self.robot_id,
            "state":         self.state.name,
            "node":          self.current_node,
            "lane":          self.current_lane_id or "—",
            "speed":         round(self.speed, 2),
            "progress":      round(self.position_on_lane * 100, 1),
            "dist_m":        round(self.total_distance, 1),
            "wait_s":        round(self.wait_time, 1),
            "replans":       self.replans,
            "estops":        self.emergency_stops,
            "lanes_done":    self.lanes_traversed,
        }
