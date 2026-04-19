# Lane-Aware Multi-Robot Traffic Control System

A complete simulation of multi-robot coordination in a structured warehouse
environment, covering lane-based routing, safety policies, deadlock detection,
and live congestion management.

---

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run the simulation (default 180-second run, 10 robots)
python main.py

# 3. Run with custom settings
python main.py --duration 300 --dt 0.05

# 4. Skip chart generation (faster)
python main.py --no-charts

# 5. Run all unit tests
python -m pytest tests/ -v
# or
python tests/test_all.py
```

Charts are saved to the `results/` folder automatically.

---

## Project Structure

```
multi_robot_traffic/
│
├── main.py                       # Entry point — runs the simulation
│
├── core/
│   ├── lane_graph.py             # Lane model + graph + Dijkstra planner
│   ├── robot.py                  # Robot agent with full state machine
│   ├── traffic_controller.py     # Central coordinator + deadlock detector
│   └── map_builder.py            # Warehouse floor plan definition
│
├── visualization/
│   └── charts.py                 # Matplotlib charts and heatmap views
│
├── tests/
│   └── test_all.py               # Unit tests for all modules
│
├── results/                      # Generated charts (created at runtime)
├── requirements.txt
└── README.md
```

---

## System Architecture

### Lane Graph (`core/lane_graph.py`)

The warehouse is modelled as a **directed graph**:
- **Nodes** = physical waypoints (intersections, entry/exit zones)
- **Edges** = lanes with full metadata

Each `Lane` stores:

| Property | Type | Description |
|---|---|---|
| `max_speed` | float (m/s) | Speed limit for this lane |
| `safety_level` | `SafetyLevel` enum | LOW / MEDIUM / HIGH |
| `lane_type` | `LaneType` enum | NORMAL / NARROW / INTERSECTION / HUMAN_ZONE |
| `length` | float (m) | Physical length |
| `directed` | bool | One-way or bidirectional |
| `requires_reservation` | bool | Critical lane flag |
| `congestion_score` | float [0–1] | Real-time occupancy ratio |
| `historical_usage` | int | Total passes recorded |

**Effective speed** is computed dynamically:
```
effective_speed = max_speed × (1 − 0.7 × congestion) × safety_factor
```
where `safety_factor` = 1.0 / 0.75 / 0.5 for LOW / MEDIUM / HIGH safety.

**Path planning** uses Dijkstra's algorithm with a travel-time cost function
that adds a congestion penalty, so robots naturally prefer less-busy routes.

---

### Robot Agent (`core/robot.py`)

Each robot is a **finite state machine**:

```
IDLE → PLANNING → MOVING ⇄ WAITING
                         ↓
                      ARRIVED
                  EMERGENCY_STOP (any time)
```

Per-tick logic:
1. If currently on a lane → advance position proportionally to time elapsed.
2. If ready for the next lane → run gate checks:
   - Is the lane **blocked**? → replan.
   - Does the lane need a **reservation**? → try to claim it; wait if denied.
   - Is there **safe following distance**? → wait if not.
   - Is the lane **too narrow**? → wait if occupied.
   - Is **congestion ≥ 90%**? → try to find an alternate route.
3. Enter the lane (record entry, set traversal timer).
4. On exit → update node, release reservation, advance to next lane.

---

### Traffic Controller (`core/traffic_controller.py`)

The controller:
- Holds the simulation clock and calls `robot.tick(dt)` for every robot each step.
- Provides the **safe-following-distance callback** used by robots (checks
  per-lane type capacity limits).
- Runs **deadlock detection** every 5 simulated seconds.
- Records the **heatmap** every tick.

#### Deadlock Detection & Resolution

A deadlock is a cyclic wait: R1 waits for R2, R2 waits for R3, R3 waits for R1.

Detection uses a **wait-for graph** + DFS cycle finding.
Resolution: the robot with the most prior replans (least fortunate) is forced
to release its lane and replan with a time penalty.

---

### Warehouse Map (`core/map_builder.py`)

```
[ENTRY]──A──[N1]──B──[N2]──C (human zone)──[N3]
              |                               |
              D (intersection*)              N3_N6
              |                               |
           [N4]──F──[N5]──G (human zone)──[N6]──N6_EXIT──[EXIT]
              |       |
              H       I
              |       |
           [N7]──J──[N8]──K──[N9]──L (human zone)──[EXIT]
                      |
                      M (narrow, one-way)
                      |
                   [N10]
```

`*` Intersection lanes D and E require reservation before entry.

---

### Visualisations (`visualization/charts.py`)

Four charts are generated after each run:

| File | Description |
|---|---|
| `lane_map.png` | Graph overlay with heat-coloured lanes (blue=cold, red=hot) |
| `heatmap_bars.png` | Per-lane bar chart: usage, congestion alerts, peak occupancy |
| `robot_metrics.png` | Per-robot grouped bars: wait time, distance, replans |
| `state_distribution.png` | Pie chart of final robot states |
| `congestion_timeline.png` | Occupancy over time for the 6 busiest lanes |

---

## Evaluation Criteria Coverage

| Criterion | How it's addressed |
|---|---|
| **Deadlock handling** | Cyclic-wait detection + forced replan with penalty |
| **Traffic efficiency** | Dijkstra with congestion-weighted cost; dynamic rerouting |
| **Safety correctness** | Per-lane speed caps, safe-following checks, human-zone limits, emergency stop |
| **Lane-aware intelligence** | Reservation system, narrow-lane exclusion, congestion replanning |
| **Scalability** | Graph-based; adding nodes/lanes/robots requires zero code changes |

---

## Performance Metrics (typical 180s run)

| Metric | Typical value |
|---|---|
| Robots arrived | 10 / 10 |
| Throughput | ~0.05 robots/s |
| Avg wait per robot | 15–40 s |
| Total replans | 5–20 |
| Emergency stops | 0 (unless triggered manually) |
| Deadlocks resolved | 0–3 |

---

## Extending the System

**Add more robots:** append entries to `get_robot_missions()` in `map_builder.py`.

**Add new lanes:** call `graph.add_lane(Lane(...))` in `build_warehouse_map()`.

**Change policies:** adjust `SAFE_FOLLOWING_FRACTION` in `TrafficController`
or the capacity table in `_is_safe_to_enter`.

**Trigger emergency stops:** call `robot.emergency_stop("reason")` from any
external event handler (sensor integration, manual override, etc.).

## Live UI Dashboard

Open `visualization/ui_dashboard.html` in any modern browser (no server needed).

### Features
- **Real-time warehouse map** — nodes, bidirectional lanes, type color-coding
- **10 animated robots** with glow trails moving across the floor plan
- **Robot status sidebar** — state badges, progress bars, speed/wait/replan stats
- **System metrics** — throughput, total wait, distance, replans
- **Interactive controls:**
  - ▶ Start / ⏸ Pause / ↺ Reset
  - Speed slider (0.1× – 8×)
  - Robot count selector (5 / 10 / 15)
  - 🔥 Heatmap mode — overlays congestion heat on lanes
  - 🚧 Block random lane — inject a blockage, watch robots reroute
  - ⚠ Emergency Stop All — halts all robots, auto-resumes after 3s
- **Deadlock detection** logged in the event console
- **Hover tooltips** on any robot for full stats
- **Event log** — timestamped stream of arrivals, replans, deadlocks, blockages
