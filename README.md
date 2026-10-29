*This project has been created as part of the 42 curriculum by segambar.*

# Fly-In

## Description

Fly-In is a Python command-line simulator for routing multiple drones through a
shared network of hubs. It plans turn-by-turn movement while respecting hub
and connection capacities, blocked zones, and restricted corridors. It prints
the schedule and opens an interactive map visualisation.

## Instructions

Requires Python 3.10+ and the packages in `requirements.txt` (including
Matplotlib). Install and run from the repository root:

```bash
python3 -m pip install -r requirements.txt
python3 main.py maps/easy/01_linear_path.txt
```

The schedule is printed to the terminal; close the visualisation window to
exit. `make run ARGS=maps/medium/03_priority_puzzle.txt` runs another map.
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

Expected terminal schedule:

```text
D1-relay
D1-goal
```

Each line lists the moves for one turn. A move token `D1-relay` means drone 1
moves to relay; the visualiser launches after the schedule is printed.

## Algorithm and implementation

The solver converts the map into a **time-expanded graph**: every usable hub
has an entry and exit node for each turn. Entry-to-exit edges enforce hub
capacity; movement edges enforce connection capacity; wait edges let drones
remain at a hub. Normal and priority links take one turn, while restricted
links are represented by a transit node and take two. Blocked hubs are omitted.

It then runs a **successive shortest augmenting path min-cost max-flow**
algorithm. A queue-based shortest-path search finds an affordable route in the
residual graph for each drone. Paired reverse edges have zero capacity and
negative cost initially; as flow is sent, they allow earlier assignments to
be undone or rerouted. The solver expands the time horizon until all drones
can reach the destination or the configured maximum is reached.

Finally, flow is decomposed into individual drone trajectories. The extracted
moves are grouped by turn and sorted by drone ID to form the printed schedule.

## Visual representation

The Matplotlib view places hubs at their configured coordinates, labels and
colors them, and shows connections and drone locations. A next-turn control
animates the schedule; drones sharing a hub are offset visually, and a drone
in a restricted corridor is drawn between its two hubs. This makes the
planned routes and timing easier to inspect.

## Resources

- [Minimum-cost flow problem](https://en.wikipedia.org/wiki/Minimum-cost_flow_problem).
- [Ford–Fulkerson algorithm](https://en.wikipedia.org/wiki/Ford%E2%80%93Fulkerson_algorithm).
- [Python documentation](https://docs.python.org/3/) and
- [Matplotlib documentation](https://matplotlib.org/stable/).

### AI usage

AI assistance was used to explain algorithm concepts, suggest code comments
and simplifications, and help draft this README. It was not used to claim
authorship of the project; the project author is responsible for reviewing
the code and documentation.
