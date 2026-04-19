"""
Lane Graph Module
=================
Defines the graph-based map where nodes are waypoints and edges are lanes.
Each lane carries metadata: speed limit, safety level, type, congestion, usage history.
"""

import heapq
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


# ─────────────────────────────────────────────
# Enums for lane classification
# ─────────────────────────────────────────────

class LaneType(Enum):
    NORMAL       = "normal"
    NARROW       = "narrow"
    INTERSECTION = "intersection"
    HUMAN_ZONE   = "human_zone"


class SafetyLevel(Enum):
    LOW    = 1   # Standard warehouse aisle
    MEDIUM = 2   # Near workstations
    HIGH   = 3   # Human-robot shared zones


# ─────────────────────────────────────────────
# Lane data structure
# ─────────────────────────────────────────────

@dataclass
class Lane:
    """
    A directed or undirected edge between two waypoints.
    Stores all metadata needed for traffic control decisions.
    """
    lane_id:         str
    from_node:       str
    to_node:         str
    max_speed:       float          # metres per second
    safety_level:    SafetyLevel
    lane_type:       LaneType
    length:          float          # metres
    directed:        bool = True    # False = bidirectional
    requires_reservation: bool = False  # True for critical lanes

    # Live state (updated at runtime)
    current_occupants: list = field(default_factory=list)  # robot IDs present
    congestion_score:  float = 0.0   # 0.0 (free) → 1.0 (jammed)
    historical_usage:  int   = 0     # total passes recorded
    is_blocked:        bool  = False  # emergency block flag

    # Reservation table: {robot_id: timestamp_reserved}
    reservation: Optional[str] = None  # robot_id that reserved this lane

    def effective_speed_limit(self) -> float:
        """
        Returns the speed a robot should actually use on this lane,
        factoring in congestion and safety.
        """
        # Congestion slows everyone down proportionally
        congestion_factor = 1.0 - (self.congestion_score * 0.7)

        # Safety zones impose additional caps
        safety_factor = {
            SafetyLevel.LOW:    1.0,
            SafetyLevel.MEDIUM: 0.75,
            SafetyLevel.HIGH:   0.5,
        }[self.safety_level]

        return max(0.1, self.max_speed * congestion_factor * safety_factor)

    def update_congestion(self):
        """Recompute congestion score based on how many robots are present."""
        # A lane becomes fully congested when 3+ robots are on it
        max_capacity = 3 if self.lane_type != LaneType.NARROW else 1
        ratio = len(self.current_occupants) / max_capacity
        self.congestion_score = min(1.0, ratio)

    def robot_enters(self, robot_id: str):
        if robot_id not in self.current_occupants:
            self.current_occupants.append(robot_id)
            self.historical_usage += 1
            self.update_congestion()

    def robot_exits(self, robot_id: str):
        if robot_id in self.current_occupants:
            self.current_occupants.remove(robot_id)
            if self.reservation == robot_id:
                self.reservation = None   # release reservation on exit
            self.update_congestion()

    def is_reserved_by_other(self, robot_id: str) -> bool:
        return self.reservation is not None and self.reservation != robot_id

    def travel_time(self) -> float:
        """Estimated seconds to traverse this lane at effective speed."""
        speed = self.effective_speed_limit()
        return self.length / speed if speed > 0 else float('inf')

    def __repr__(self):
        return (f"Lane({self.lane_id}: {self.from_node}→{self.to_node} "
                f"spd={self.effective_speed_limit():.1f} "
                f"cong={self.congestion_score:.2f})")


# ─────────────────────────────────────────────
# The Lane Graph
# ─────────────────────────────────────────────

class LaneGraph:
    """
    Graph where nodes are named waypoints and edges are Lane objects.
    Supports directed and undirected edges.
    Provides Dijkstra-based path planning that respects lane conditions.
    """

    def __init__(self):
        self.nodes: set[str] = set()
        self.lanes: dict[str, Lane] = {}          # lane_id → Lane
        self.adjacency: dict[str, list[str]] = {} # node → [lane_ids leaving that node]

    def add_node(self, node_id: str):
        self.nodes.add(node_id)
        if node_id not in self.adjacency:
            self.adjacency[node_id] = []

    def add_lane(self, lane: Lane):
        """Register a lane (and its reverse if undirected)."""
        self.add_node(lane.from_node)
        self.add_node(lane.to_node)
        self.lanes[lane.lane_id] = lane
        self.adjacency[lane.from_node].append(lane.lane_id)

        if not lane.directed:
            # Create reverse lane automatically
            reverse_id = lane.lane_id + "_rev"
            reverse = Lane(
                lane_id=reverse_id,
                from_node=lane.to_node,
                to_node=lane.from_node,
                max_speed=lane.max_speed,
                safety_level=lane.safety_level,
                lane_type=lane.lane_type,
                length=lane.length,
                directed=True,  # stored as directed internally
                requires_reservation=lane.requires_reservation,
            )
            self.lanes[reverse_id] = reverse
            self.adjacency[lane.to_node].append(reverse_id)

    def get_lane(self, lane_id: str) -> Optional[Lane]:
        return self.lanes.get(lane_id)

    def lanes_from(self, node: str) -> list[Lane]:
        """All lanes departing from a given node."""
        return [self.lanes[lid] for lid in self.adjacency.get(node, [])]

    def find_path(self, start: str, goal: str,
                  avoid_blocked: bool = True,
                  robot_id: str = "") -> Optional[list[str]]:
        """
        Dijkstra shortest path by travel time.
        Returns list of lane_ids forming the path, or None if unreachable.

        Avoids:
        - Blocked lanes
        - Lanes reserved by other robots
        - Heavily congested lanes (score ≥ 0.9) when alternatives exist
        """
        if start == goal:
            return []

        # Priority queue: (cost, current_node, path_so_far)
        heap = [(0.0, start, [])]
        visited: dict[str, float] = {}

        while heap:
            cost, node, path = heapq.heappop(heap)

            if node in visited and visited[node] <= cost:
                continue
            visited[node] = cost

            if node == goal:
                return path

            for lane in self.lanes_from(node):
                # Skip blocked lanes
                if avoid_blocked and lane.is_blocked:
                    continue
                # Skip lanes reserved by someone else
                if lane.is_reserved_by_other(robot_id):
                    continue
                # Add heavy congestion penalty (don't skip entirely — robot might have no choice)
                congestion_penalty = 1.0 + lane.congestion_score * 2.0
                new_cost = cost + lane.travel_time() * congestion_penalty
                next_node = lane.to_node

                if next_node not in visited or visited.get(next_node, float('inf')) > new_cost:
                    heapq.heappush(heap, (new_cost, next_node, path + [lane.lane_id]))

        return None  # No path found

    def all_lane_ids(self) -> list[str]:
        return list(self.lanes.keys())

    def __repr__(self):
        return f"LaneGraph({len(self.nodes)} nodes, {len(self.lanes)} lanes)"
