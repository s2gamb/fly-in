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
Turn 1: D1-wait
```

Each click produces one turn. In the current wait-only baseline, the turn log
shows `D1-wait` to indicate drone 1 stays in place.

## Algorithm and implementation

The current simulator is a minimal wait-only baseline: each **Next Turn** click
advances the turn counter and leaves every drone in its current hub. This
provides a simple starting point for adding movement behavior incrementally.
The simulation stops after 100 turns or immediately if all drones are already
at the goal.

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
