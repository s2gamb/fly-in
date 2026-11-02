*This project has been created as part of the 42 curriculum by segambar.*

# Fly-In

## Description

Fly-In is a Python command-line simulator for exploring turn-by-turn drone
movement through a shared network of hubs. It prints each turn's actions and
opens an interactive map visualisation.

## Instructions

Requires Python 3.10+ and the packages in `requirements.txt` (including
Matplotlib). Install and run from the repository root:

```bash
python3 -m pip install -r requirements.txt
python3 main.py maps/easy/01_linear_path.txt
```

The map opens in an interactive window. Click **Next Turn** to schedule and
execute one turn; drones glide smoothly to their new positions, and that
turn's moves are printed in the terminal. Close the window to exit.
`make run ARGS=maps/medium/03_priority_puzzle.txt` runs another map.
`make lint` runs the configured Flake8 and mypy checks.

### Example

Input map:

```text
nb_drones: 1
start_hub: start 0 0
hub: relay 1 0
end_hub: goal 2 0
connection: start-relay
connection: relay-goal
```

Example turn output:

```text
Turn 1: D1-relay
Turn 2: D1-goal
```

Each click produces one turn. Moving into a restricted hub takes two turns:
the drone is shown halfway along the connection on the first turn and arrives
at the hub on the second. Routes are selected using these movement costs.

## Algorithm and implementation

The map is treated as an undirected graph: hubs are vertices and connections
are edges. Each call to `step()` advances the simulation by one turn and
processes drones in ascending ID order. A drone already at the goal stays
there. Other drones attempt to move along a minimum-cost route to the goal.

### Route selection

Before moving a drone, the solver searches backward from the goal to calculate
the minimum remaining movement cost for every reachable hub. It uses
Dijkstra's algorithm with a min-heap (`heapq`), because entering a restricted
hub has a different cost from entering other hubs:

| Destination hub zone | Movement cost |
| --- | ---: |
| `normal` or `priority` | 1 turn |
| `restricted` | 2 turns |

The cost applies when entering the destination hub. For example, a route with
two normal hubs costs two turns, while a route that enters a restricted hub
and then a normal hub costs three. The solver considers only adjacent hubs
whose cost plus their remaining cost equals the current hub's minimum cost.
This keeps a drone on a shortest-by-turns route. If there are multiple
equal-cost next hops, they are considered in the order their connections were
defined in the map.

`priority` currently has the same movement cost as `normal`; it does not
provide a tie-break preference. `blocked` hubs are excluded from route search,
so drones cannot route through or enter them. If no unblocked route to the goal
exists, route selection raises a no-route error.

### Turn execution and capacity

For each drone, the solver tries the minimum-cost next hops in order:

1. It checks that the connection has a free slot this turn
   (`max_link_capacity`) and that the destination hub has room
   (`max_drones`).
2. If a normal hub is available, the drone moves there immediately.
3. If the destination is restricted, the drone enters transit on that
   connection. Its displayed location is recorded as `source-target`, so the
   visualiser places it halfway between the hubs. On a later turn it completes
   the move into the restricted hub, subject to the connection's per-turn
   capacity.
4. If no eligible next hop has capacity, the drone waits for this turn.

The destination hub's capacity is reserved when a restricted move starts, so
another drone cannot claim its last slot while the first drone is in transit.
Connection usage is reset after all drones have been processed, making link
capacity a per-turn limit. Hub occupancy is maintained as drones move into and
out of hubs.

This is a greedy, sequential scheduler: earlier drone IDs get first access to
available capacity. Route planning uses movement costs, but does not optimize
the overall schedule for all drones together. The simulation ends when all
drones reach the goal or after 100 turns.

## Visual representation

The Matplotlib view places hubs at their configured coordinates, labels and
colors them, and shows connections and current drone locations. Each click
advances the turn and animates any drone movement;
drones sharing a hub are offset visually, and a drone in a restricted corridor
is drawn between its two hubs. Restricted hubs have a dashed ring and an `R`
marker; hub colors are softened to pastels, and drones sharing a hub cluster
closer to its center. Connections with a capacity other than one show a `cap N`
label at the link midpoint. Non-start/end hubs with a configured capacity other
than one show a `cap N` label below the hub.

## Resources

- [Python documentation](https://docs.python.org/3/) and
- [Matplotlib documentation](https://matplotlib.org/stable/).

### AI usage

AI assistance was used to explain algorithm concepts, suggest code comments
and simplifications, and help draft this README. It was not used to claim
authorship of the project; the project author is responsible for reviewing
the code and documentation.
