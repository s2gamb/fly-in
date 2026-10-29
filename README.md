*This project has been created as part of the 42 curriculum by segambar.*

# Fly-In

## Description

Fly-In is a Python command-line simulator for routing multiple drones through a
shared network of hubs. It simulates turn-by-turn movement while respecting
hub and connection capacities, blocked zones, and restricted corridors. It
prints the moves and opens an interactive map visualisation.

## Instructions

Requires Python 3.10+ and the packages in `requirements.txt` (including
Matplotlib). Install and run from the repository root:

```bash
python3 -m pip install -r requirements.txt
python3 main.py maps/easy/01_linear_path.txt
```

The map opens in an interactive window. Click **Next Turn** to schedule and
execute one turn; that turn's moves are also printed in the terminal. Close
the window to exit. `make run ARGS=maps/medium/03_priority_puzzle.txt` runs
another map.
`make lint` runs the configured Flake8 and mypy checks.

The first routing iteration is a random walk. Pass `--seed` to reproduce a
particular run:

```bash
python3 main.py --seed 42 maps/easy/03_basic_capacity.txt
```

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

Example successful sequence of clicks:

```text
D1-relay
D1-goal
```

Each click produces one turn of moves. A token `D1-relay` means drone 1 moves
to relay.

## Algorithm and implementation

The first routing iteration is intentionally simple: clicking **Next Turn**
asks each drone to pick a random neighboring hub from its current location.
The chosen moves are applied immediately, then the visualizer redraws the new
state. Blocked zones are excluded, hub and connection capacities are enforced,
and restricted zones take two turns to cross. Drones take turns in rotating ID
order, so capacity conflicts can make a drone wait. A run stops when all
drones reach the goal, no moves are possible, or 100 turns have run; the seed
option makes a random run reproducible.

## Visual representation

The Matplotlib view places hubs at their configured coordinates, labels and
colors them, and shows connections and current drone locations. Each click
advances the simulation before redrawing; drones sharing a hub are offset
visually, and a drone in a restricted corridor is drawn between its two hubs.

## Resources

- [Python documentation](https://docs.python.org/3/) and
- [Matplotlib documentation](https://matplotlib.org/stable/).

### AI usage

AI assistance was used to explain algorithm concepts, suggest code comments
and simplifications, and help draft this README. It was not used to claim
authorship of the project; the project author is responsible for reviewing
the code and documentation.
