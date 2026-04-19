"""
Map Builder
===========
Defines the warehouse floor plan as a LaneGraph.

Layout (top-down view):

    [ENTRY]──A──[N1]──B──[N2]──C──[N3]
                 |              |
                 D              E
                 |              |
              [N4]──F──[N5]──G──[N6]
                 |         |
                 H         I
                 |         |
              [N7]──J──[N8]──K──[N9]──L──[EXIT]
                              |
                              M (narrow service corridor)
                              |
                           [N10]

Nodes legend:
  ENTRY / EXIT  — start and end zones
  N1–N3         — top aisle waypoints
  N4–N6         — middle aisle waypoints
  N7–N10        — bottom aisle waypoints + service bay

Special lanes:
  E             — intersection (requires reservation)
  D             — intersection (requires reservation)
  M             — narrow service corridor
  C, G, L       — human zones (low speed)
"""

from core.lane_graph import LaneGraph, Lane, LaneType, SafetyLevel


def build_warehouse_map() -> LaneGraph:
    """
    Construct and return the warehouse LaneGraph.
    All lengths are in metres; speeds in m/s.
    """
    g = LaneGraph()

    # ── Nodes ──────────────────────────────────────────────────────────────
    nodes = [
        "ENTRY", "N1", "N2", "N3",
        "N4",    "N5", "N6",
        "N7",    "N8", "N9", "N10",
        "EXIT"
    ]
    for n in nodes:
        g.add_node(n)

    # ── Top aisle (normal, bidirectional) ──────────────────────────────────
    g.add_lane(Lane("A", "ENTRY", "N1", max_speed=2.0, length=10.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    g.add_lane(Lane("B", "N1", "N2", max_speed=2.0, length=12.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    g.add_lane(Lane("C", "N2", "N3", max_speed=1.0, length=8.0,
                    safety_level=SafetyLevel.HIGH,
                    lane_type=LaneType.HUMAN_ZONE, directed=False))

    # ── Vertical connectors (intersections — require reservation) ──────────
    g.add_lane(Lane("D", "N1", "N4", max_speed=1.5, length=15.0,
                    safety_level=SafetyLevel.MEDIUM,
                    lane_type=LaneType.INTERSECTION,
                    directed=False, requires_reservation=True))

    g.add_lane(Lane("E", "N3", "N6", max_speed=1.5, length=15.0,
                    safety_level=SafetyLevel.MEDIUM,
                    lane_type=LaneType.INTERSECTION,
                    directed=False, requires_reservation=True))

    # ── Middle aisle ───────────────────────────────────────────────────────
    g.add_lane(Lane("F", "N4", "N5", max_speed=2.0, length=12.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    g.add_lane(Lane("G", "N5", "N6", max_speed=1.0, length=8.0,
                    safety_level=SafetyLevel.HIGH,
                    lane_type=LaneType.HUMAN_ZONE, directed=False))

    # ── Vertical connectors middle→bottom ─────────────────────────────────
    g.add_lane(Lane("H", "N4", "N7", max_speed=2.0, length=15.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    g.add_lane(Lane("I", "N5", "N8", max_speed=2.0, length=15.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    # ── Bottom aisle ───────────────────────────────────────────────────────
    g.add_lane(Lane("J", "N7", "N8", max_speed=2.5, length=12.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    g.add_lane(Lane("K", "N8", "N9", max_speed=2.5, length=12.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    g.add_lane(Lane("L", "N9", "EXIT", max_speed=1.0, length=10.0,
                    safety_level=SafetyLevel.HIGH,
                    lane_type=LaneType.HUMAN_ZONE, directed=False))

    # ── Narrow service corridor (one-way) ─────────────────────────────────
    g.add_lane(Lane("M", "N8", "N10", max_speed=0.5, length=6.0,
                    safety_level=SafetyLevel.HIGH,
                    lane_type=LaneType.NARROW, directed=True))

    # ── N3 shortcut back to N6 (for top-right robots) ─────────────────────
    g.add_lane(Lane("N3_N6", "N3", "N6", max_speed=1.2, length=5.0,
                    safety_level=SafetyLevel.MEDIUM,
                    lane_type=LaneType.NORMAL, directed=False))

    # ── N6 → EXIT shortcut ────────────────────────────────────────────────
    g.add_lane(Lane("N6_EXIT", "N6", "EXIT", max_speed=1.5, length=18.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    # ── N7 → EXIT long path ───────────────────────────────────────────────
    g.add_lane(Lane("N7_EXIT", "N7", "EXIT", max_speed=1.5, length=25.0,
                    safety_level=SafetyLevel.LOW,
                    lane_type=LaneType.NORMAL, directed=False))

    return g


def get_robot_missions() -> list[dict]:
    """
    Define start and goal for each of the 8+ robots.
    Deliberately mix routes to create interesting traffic patterns.
    """
    return [
        {"id": "R01", "start": "ENTRY", "goal": "EXIT"},
        {"id": "R02", "start": "ENTRY", "goal": "N10"},
        {"id": "R03", "start": "N3",    "goal": "N7"},
        {"id": "R04", "start": "N7",    "goal": "ENTRY"},
        {"id": "R05", "start": "N5",    "goal": "EXIT"},
        {"id": "R06", "start": "ENTRY", "goal": "N6"},
        {"id": "R07", "start": "N9",    "goal": "N1"},
        {"id": "R08", "start": "N4",    "goal": "EXIT"},
        {"id": "R09", "start": "N2",    "goal": "N8"},
        {"id": "R10", "start": "ENTRY", "goal": "N3"},
    ]
