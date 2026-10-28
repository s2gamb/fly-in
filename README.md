*This project has been created as part of the 42 curriculum by segambar.*

# Fly-In: Multi-Drone Space-Time Routing Engine

Fly-In is an advanced multi-agent pathfinding (MAPF) and discrete event simulation engine written in Python. It plans optimal, conflict-free flight paths for fleets of autonomous drones navigating through constrained topological airspace networks from an origin hub (`start_hub`) to a destination hub (`end_hub`).

The system models complex real-world operational constraints including hub capacities, connection throughput limits, terrain zone penalties, and multi-turn transit corridors, coupling mathematical optimization with an interactive 2D spatial visualizer.

---

## Table of Contents

1. [Description](#description)
2. [Instructions](#instructions)
   - [Prerequisites & Dependencies](#prerequisites--dependencies)
   - [Execution](#execution)
   - [Makefile Commands](#makefile-commands)
   - [Project Structure](#project-structure)
3. [Algorithm Choices & Implementation Strategy](#algorithm-choices--implementation-strategy)
   - [Why Static Pathfinding Fails](#why-static-pathfinding-fails)
   - [Space-Time Network (Time-Expanded Graph)](#space-time-network-time-expanded-graph)
   - [Min-Cost Max-Flow (MCMF) Formulation](#min-cost-max-flow-mcmf-formulation)
   - [Iterative Deepening Time Horizon](#iterative-deepening-time-horizon)
   - [Flow Deconvolution & Trajectory Extraction](#flow-deconvolution--trajectory-extraction)
4. [Visual Representation Features](#visual-representation-features)
   - [Interactive Spatial Visualizer](#interactive-spatial-visualizer)
   - [Multi-Drone Dispersion & Collision Rendering](#multi-drone-dispersion--collision-rendering)
   - [Transit State Visualization](#transit-state-visualization)
   - [UX Value & Debugging Ergonomics](#ux-value--debugging-ergonomics)
5. [Input Format & Expected Output](#input-format--expected-output)
   - [Map Configuration Syntax](#map-configuration-syntax)
   - [Example Input](#example-input)
   - [Expected Output & Token Semantics](#expected-output--token-semantics)
6. [Resources & AI Usage](#resources--ai-usage)
   - [References & Literature](#references--literature)
   - [AI Usage Declaration](#ai-usage-declaration)

---

## Description

In modern automated delivery networks, air traffic management requires coordinating multiple autonomous aircraft over shared topological infrastructure while avoiding mid-air collisions, congestion at hubs, and queue overflow.

Inspired by 42's classical *Lem-in* problem and expanded to modern multi-agent systems, **Fly-In** solves this logistical coordination challenge:
- **Agents**: $N$ individual drones ($D_1, D_2, \dots, D_N$) starting simultaneously at `start_hub`.
- **Target**: Deliver all $N$ drones to `end_hub` in the minimum number of simulation turns.
- **Node Capacities**: Each hub $u$ enforces a concurrent drone capacity (`max_drones`). Start and end hubs have infinite capacity.
- **Edge Capacities**: Each bidirectional connection enforces a concurrent link capacity (`max_link_capacity`).
- **Zone Classifications**:
  - `normal`: Standard 1-turn traversal, standard cost.
  - `priority`: High-speed airways (1 turn traversal, half cost to incentivize usage).
  - `restricted`: Complex terrain requiring 2 turns to enter and land, occupying transit airway airspace.
  - `blocked`: No-fly zones (completely inaccessible).

The program outputs turn-by-turn discrete movement instructions complying with strict standard formatting, followed by launching an interactive visual interface.

---

## Instructions

### Prerequisites & Dependencies

- **Python 3.10+**
- **Matplotlib** (used for the 2D visualizer)

Install the required Python package:
```bash
pip install matplotlib
```

### Execution

Run the simulation by passing any valid map file to `main.py`:
```bash
python3 main.py maps/easy/01_linear_path.txt
```

You can also test maps across different difficulty tiers:
```bash
# Easy linear map
python3 main.py maps/easy/01_linear_path.txt

# Forking paths with capacity management
python3 main.py maps/easy/02_simple_fork.txt

# Priority vs. restricted zone trade-off
python3 main.py maps/medium/03_priority_puzzle.txt

# Challenger level: The Impossible Dream (solves in ~43 turns)
python3 main.py maps/challenger/01_the_impossible_dream.txt
```

### Makefile Commands

The repository includes a comprehensive `Makefile` automating development tasks:
```bash
# Install dependencies (matplotlib, flake8, mypy)
make install

# Run simulation with default map (maps/easy/01_linear_path.txt)
make run

# Run simulation with a custom map
make run ARGS=maps/medium/03_priority_puzzle.txt

# Run main script in debug mode using Python's built-in debugger (pdb)
make debug

# Run linting and type checking (flake8 and mypy with required flags)
make lint

# Run strict linting and type checking (flake8 and mypy --strict)
make lint-strict

# Clean temporary Python bytecode and cache directories (__pycache__, .mypy_cache)
make clean
```

### Project Structure

```text
.
├── Makefile                     # Build & automation rules (install, run, debug, clean, lint)
├── README.md                    # Project documentation
├── main.py                      # CLI entry point and simulation runner
├── mypy.ini                     # Type checker configuration
├── requirements.txt             # Project and development dependencies
├── maps/                        # Challenge and benchmark maps
│   ├── easy/                    # Introductory linear and branching maps
│   ├── medium/                  # Dead-ends, circular loops, priority puzzles
│   ├── hard/                    # High-density capacity bottlenecks and mazes
│   └── challenger/              # Algorithmic limit stress test (The Impossible Dream)
└── src/
    ├── __init__.py
    ├── parser.py                # Map file parser and strict syntax validator
    ├── graph.py                 # Time-Expanded Graph (TEG) and MCMF residual network
    ├── solver.py                # Discrete-event drone solver & trajectory decoder
    └── visualiser.py            # Matplotlib interactive spatial GUI
```

---

## Algorithm Choices & Implementation Strategy

### Why Static Pathfinding Fails

Traditional single-agent pathfinding algorithms (such as Dijkstra, static $A^*$, or basic BFS) determine the shortest spatial path between two nodes. When multiple agents share the same network, static algorithms fail due to:
1. **Dynamic Collisions**: Drones scheduled on the same hub or link simultaneously exceed capacity.
2. **Greedy Traps**: A drone taking the shortest physical path might block a high-throughput corridor, forcing subsequent drones into lengthy detours.
3. **Dead-ends & Capacity Queues**: Drones cannot wait or adjust speed without a temporal dimension.

### Space-Time Network (Time-Expanded Graph)

To solve multi-agent routing globally and avoid conflicts, the spatial graph $G = (V, E)$ is unfolded across discrete time intervals $t \in [0, T_{\max}]$, producing a **Time-Expanded Graph (TEG)**:

```text
Time t:    [ u_in(t)  ] ---> [ u_out(t) ] ===== wait edge =====> [ u_in(t+1) ]
                                   |                                    ^
                             movement edge                              |
                                   v                                    |
Time t+1:  [ v_in(t+1) ] ---> [ v_out(t+1) ]                     [ v_in(t+1) ]
```

#### Node Splitting for Hub Capacity
Each physical hub $u \in V$ at time $t$ is split into two internal nodes:
- An entry node: $u_{\text{in}}[t]$
- An exit node: $u_{\text{out}}[t]$
- A directed edge $u_{\text{in}}[t] \to u_{\text{out}}[t]$ is created with:
  $$\text{capacity} = \begin{cases} \text{nb\_drones} & \text{if } u \in \{\text{start\_hub}, \text{end\_hub}\} \\ \text{hub.max\_drones} & \text{otherwise} \end{cases}, \quad \text{cost} = 0.0$$
This structural split ensures that no more than `max_drones` can occupy hub $u$ at time step $t$.

#### Wait Edges (Hovering / Queuing)
A drone can remain stationary at hub $u$ across consecutive turns. This is represented by a directed temporal edge:
$$u_{\text{out}}[t] \to u_{\text{in}}[t+1], \quad \text{capacity} = \text{hub.max\_drones}, \quad \text{cost} = 1.0$$

#### Spatial Movement Edges & Zone Semantics
For each undirected connection $(u, v)$ with link capacity $C$:
- **Normal Zone**: Traversing to $v$ takes 1 turn.
  $$u_{\text{out}}[t] \to v_{\text{in}}[t+1], \quad \text{capacity} = C, \quad \text{cost} = 1.0$$
- **Priority Zone**: High-speed airway incentivizing flow.
  $$u_{\text{out}}[t] \to v_{\text{in}}[t+1], \quad \text{capacity} = C, \quad \text{cost} = 0.5$$
- **Restricted Zone**: Incurs a 2-turn movement penalty. An intermediate transit node is introduced:
  $$u_{\text{out}}[t] \to \text{transit}_{u \to v}[t+1], \quad \text{capacity} = C, \quad \text{cost} = 1.0$$
  $$\text{transit}_{u \to v}[t+1] \to v_{\text{in}}[t+2], \quad \text{capacity} = C, \quad \text{cost} = 1.0$$
- **Blocked Zone**: Inaccessible obstacles. Hubs marked as `blocked` and all incident connections are filtered out during graph assembly.

### Min-Cost Max-Flow (MCMF) Formulation

The flow network connects:
- A super-source node connected to $\text{start\_hub\_out}[0]$ with capacity $N$ (`nb_drones`) and cost $0$.
- Each time slice of $\text{end\_hub\_out}[t]$ connected to a super-sink node with capacity $N$ and cost $0$.

The solver uses the **Successive Shortest Path (SSP)** algorithm backed by **SPFA (Shortest Path Faster Algorithm)**:
1. In each iteration, SPFA identifies the least-cost augmenting path in the residual graph from the super-source to the super-sink.
2. SPFA accommodates negative-cost residual backward edges created during flow cancellation without falling into infinite negative cycles.
3. Flow is augmented 1 unit at a time until the cumulative flow equals $N$ (all drones reach the destination).
4. Because each unit of flow represents a drone's world-line through space and time, integer flow conservation guarantees zero node or link capacity violations.

### Iterative Deepening Time Horizon

The exact makespan $T^*$ needed for all drones to reach `end_hub` is not known a priori:
- If $T_{\max}$ is chosen too small, the target flow $N$ cannot be routed.
- If $T_{\max}$ is excessively large, graph construction and SPFA residual search incur unnecessary memory and CPU overhead.

Fly-In solves this with **dynamic horizon expansion**:
1. Pre-checks network reachability via BFS; if `end_hub` is disconnected from `start_hub`, it immediately terminates with an informative error.
2. Initializes the horizon:
   $$T_{\text{init}} = \max\left(\text{default\_turns}, |\text{hubs}| + N\right)$$
3. Attempts to route flow $N$. If the network cannot push $N$ units of flow, $T_{\max}$ expands by $+30$ turns iteratively until a solution is found or the safety ceiling (`max_allowed_turns = 500`) is reached.

### Flow Deconvolution & Trajectory Extraction

Once the optimal flow is established in the time-expanded graph, the continuous flow matrix is decomposed into discrete paths for drones $D_1, \dots, D_N$:
1. For each drone $i \in \{1, \dots, N\}$, a flow path is traced from $\text{start\_hub\_out}[0]$ to $\text{end\_hub}$ by iteratively consuming unit flow on traversed edges.
2. Moves are converted into chronological turn events:
   - Hub arrivals: `D<id>-<hub_name>`
   - Restricted transit entries: `D<id>-<source>-<target>`
   - Hover turns: internal wait edges emit no token, as the drone remains parked at its current hub.
3. Turn actions are sorted deterministically by drone ID and formatted into single-line outputs per simulation turn.

---

## Visual Representation Features

To provide visual insight beyond terminal logs, Fly-In integrates an interactive graphical interface built on Matplotlib (`src/visualiser.py`).

```text
┌────────────────────────────────────────────────────────┐
│  Fly-In Spatial Visualizer                             │
│                                                        │
│       [start] ─── (D1) ───> [bottleneck] ───> [goal]   │
│         (D2)                                           │
│                                                        │
│  Grid: 1.0 unit grid lines                             │
│  Aspect Ratio: 1:1 true coordinate geometry            │
│                                                        │
│                                      [ Next Turn ▶ ]   │
└────────────────────────────────────────────────────────┘
```

### Interactive Spatial Visualizer
- **True Coordinate Rendering**: Hubs are plotted at their exact $(x, y)$ coordinates specified in the map file with an equal 1:1 aspect ratio.
- **Topological Connections**: Airway links are rendered with uniform width and contrast connecting adjacent hubs.
- **Hub Color Coding**: Respects user-defined color directives from map metadata (e.g. `[color=blue]`, `[color=red]`, `[color=green]`).

### Multi-Drone Dispersion & Collision Rendering
In high-throughput hubs where `max_drones > 1`, rendering multiple drones at the exact center would occlude labels.
Fly-In calculates dynamic circular offsets:
$$\theta_k = \frac{2\pi \cdot k}{K}, \quad x_{\text{offset}} = r \cos(\theta_k), \quad y_{\text{offset}} = r \sin(\theta_k)$$
where $K$ is the number of co-occupants at that turn. Each drone receives a distinct blue badge containing its white numerical ID ($1, \dots, N$), ensuring all agents remain visible and legible.

### Transit State Visualization
When drones enter a 2-turn `restricted` zone, they are physically mid-flight along the connecting airway during the first turn. The visualizer places the drone badge at the exact geometric midpoint:
$$x_{\text{mid}} = \frac{x_u + x_v}{2}, \quad y_{\text{mid}} = \frac{y_u + y_v}{2}$$
This accurately reflects that the drone is occupying link capacity rather than hub capacity.

### UX Value & Debugging Ergonomics
- **"Next Turn" Control Widget**: An interactive GUI button allows users, evaluators, and recruiters to step turn-by-turn through the mission.
- **Verification of Invariants**: Confirms that node and edge capacities are never violated and highlights waiting drones holding at hubs while priority drones pass through.
- **Seamless Headless Fallback**: In non-interactive environments (CI/CD, headless terminals without display servers), the engine cleanly suppresses GUI display errors and completes without crashing.

---

## Input Format & Expected Output

### Map Configuration Syntax

Map files are plain text files defining the problem instance. Comments begin with `#` and empty lines are ignored.

```ini
# First directive must specify drone fleet size
nb_drones: <positive_integer>

# Hub declarations: <type>: <name> <x> <y> [optional_attributes]
start_hub: <name> <x> <y> [color=<color>]
hub: <name> <x> <y> [zone=<normal|restricted|priority|blocked>] [max_drones=<int>] [color=<color>]
end_hub: <name> <x> <y> [color=<color>]

# Connection declarations: connection: <hub1>-<hub2> [max_link_capacity=<int>]
connection: <source>-<target> [max_link_capacity=<int>]
```

### Example Input

File: `maps/easy/01_linear_path.txt`
```ini
# Easy Level 1: Simple linear path
nb_drones: 2

start_hub: start 0 0 [color=green]
hub: waypoint1 1 0 [color=blue]
hub: waypoint2 2 0 [color=blue]
end_hub: goal 3 0 [color=red]

connection: start-waypoint1
connection: waypoint1-waypoint2
connection: waypoint2-goal
```

### Expected Output & Token Semantics

When running `python3 main.py maps/easy/01_linear_path.txt`:

```text
D2-waypoint1
D1-waypoint1 D2-waypoint2
D1-waypoint2 D2-goal
D1-goal
```

#### Token Breakdown
- **Turn 1**: Drone `D2` moves from `start` to `waypoint1`. Drone `D1` remains at `start` due to `max_drones=1` capacity at `waypoint1`.
- **Turn 2**: Drone `D2` advances to `waypoint2`, freeing `waypoint1` for drone `D1`.
- **Turn 3**: Drone `D2` reaches `goal`, and `D1` advances to `waypoint2`.
- **Turn 4**: Drone `D1` reaches `goal`. Total simulation completed in 4 turns.

For maps with **restricted zones** (such as `maps/medium/03_priority_puzzle.txt`):
- `D4-start-slow_path1`: Drone 4 enters restricted transit between `start` and `slow_path1` (Turn 1).
- `D4-slow_path1`: Drone 4 arrives at `slow_path1` (Turn 2).

---

## Resources & AI Usage

### References & Literature

- **Multi-Agent Path Finding (MAPF)**:
  - Stern, R., et al. *"Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks"* (2019).
  - Sharon, G., et al. *"Conflict-Based Search for Optimal Multi-Agent Pathfinding"* (Artificial Intelligence, 2015).
- **Network Flow & Time-Expanded Networks**:
  - Ford, L. R., & Fulkerson, D. R. *"Flows in Networks"*, Princeton University Press.
  - Ahuja, R. K., Magnanti, T. L., & Orlin, J. B. *"Network Flows: Theory, Algorithms, and Applications"*, Prentice Hall.
  - Köhler, E., Langkau, K. F., & Skutella, M. *"Time-Expanded Networks for Dynamic Network Flows"*.
- **Algorithms**:
  - Shortest Path Faster Algorithm (SPFA) — Yen (1970) / F. Ding (1994).
  - Edmonds-Karp and Busacker-Gowen algorithms for Minimum-Cost Maximum-Flow.
- **Visualization & UI**:
  - [Matplotlib Event Handling and Widgets Documentation](https://matplotlib.org/stable/users/explain/figure/event_handling.html)

### AI Usage Declaration

In accordance with academic integrity guidelines, generative AI was utilized during the development of this project for specific tasks:
1. **Mathematical Formulation & Algorithm Brainstorming**:
   - Assisting in modeling heterogeneous zones (`normal`, `priority`, `restricted`, `blocked`) as unit costs and multi-layer transit node chains in the Time-Expanded Graph.
2. **Parser Edge-Case Hardening**:
   - Refining regular expressions and validation logic in `src/parser.py` (e.g., verifying canonical undirected edge uniqueness, bracket attribute syntax, and preventing self-referential links).
3. **Refactoring & Code Quality**:
   - Adding type hints (`typing` / Python 3.10 dataclasses) and structuring modular separation across `parser.py`, `graph.py`, `solver.py`, and `visualiser.py`.
4. **Documentation & Benchmarks**:
   - Structuring comprehensive documentation, benchmark validation across maps, and explaining token syntax.
