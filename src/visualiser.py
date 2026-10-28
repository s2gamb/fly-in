import math
import sys
from typing import Any

import matplotlib.pyplot as plt  # type: ignore[import-untyped]
from matplotlib.ticker import MultipleLocator  # type: ignore[import-untyped]

from src.parser import MapData
from src.solver import SimulationResult


class Visualiser:
    def __init__(self, map_data: MapData, result: SimulationResult) -> None:
        self.map_data = map_data
        self.result = result
        self.figsize: tuple[float, float] = (12.0, 9.0)
        self.fig: Any = None
        self.ax: Any = None
        self.current_turn: int = 0
        self.drone_locs: dict[int, dict[int, str]] = (
            self._index_drone_locations()
        )
        self.btn_next: Any = None
        self.drone_artists: list[tuple[Any, Any]] = []

    def _index_drone_locations(self) -> dict[int, dict[int, str]]:
        loc_map: dict[int, dict[int, str]] = {}
        assert self.map_data.start_hub is not None
        for drone_id in range(1, self.map_data.nb_drones + 1):
            loc_map[drone_id] = {}
            traj = self.result.paths[drone_id]
            traj_dict = dict(traj)
            curr_loc = self.map_data.start_hub.name
            for turn in range(self.result.total_turns + 1):
                if turn in traj_dict:
                    curr_loc = traj_dict[turn]
                loc_map[drone_id][turn] = curr_loc
        return loc_map

    def _get_drone_coordinates(
        self, drone_id: int, turn: int, hub_occupants: dict[str, list[int]]
    ) -> tuple[float, float]:
        loc = self.drone_locs.get(drone_id, {}).get(turn, "")
        hubs = self.map_data.hubs
        if loc in hubs:
            hub = hubs[loc]
            occupants = hub_occupants.get(loc, [drone_id])
            num_occ = len(occupants)
            if num_occ <= 1:
                return float(hub.x), float(hub.y)
            idx = occupants.index(
                drone_id
            )  # if drone_id not in occupants, this will raise ValueError
            angle = (2.0 * math.pi * idx) / num_occ
            radius = 0.3
            x_offset = radius * math.cos(angle)
            y_offset = radius * math.sin(angle)
            return float(hub.x) + x_offset, float(hub.y) + y_offset
        elif "-" in loc:
            parts = loc.split("-")
            u_name, v_name = parts[0], parts[1]
            if u_name in hubs and v_name in hubs:
                u_hub = hubs[u_name]
                v_hub = hubs[v_name]
                return (float(u_hub.x) + float(v_hub.x)) / 2.0, (
                    float(u_hub.y) + float(v_hub.y)
                ) / 2.0
        return 0.0, 0.0

    def render(self) -> None:
        self.fig = plt.figure(figsize=self.figsize, dpi=100)
        self.ax = self.fig.add_axes([0.08, 0.18, 0.74, 0.74])
        self.ax.set_facecolor("#f8fafc")
        hubs = self.map_data.hubs
        xs = [h.x for h in hubs.values()]
        ys = [h.y for h in hubs.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        margin_x = max(1.2, (max_x - min_x) * 0.18)
        margin_y = max(1.2, (max_y - min_y) * 0.18)
        self.ax.set_xlim(min_x - margin_x, max_x + margin_x)
        self.ax.set_ylim(min_y - margin_y, max_y + margin_y)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.xaxis.set_major_locator(MultipleLocator(1))
        self.ax.yaxis.set_major_locator(MultipleLocator(1))
        self.ax.grid(True, linestyle="--", alpha=0.4, color="#cbd5e1")
        for conn in self.map_data.connections:
            hub1 = hubs[conn.source]
            hub2 = hubs[conn.target]
            self.ax.plot(
                [hub1.x, hub2.x],
                [hub1.y, hub2.y],
                # color="#94a3b8",
                color="#000000",
                linewidth=3.5,
                zorder=1,
            )
        for name, hub in hubs.items():
            color = hub.color if hub.color else "#000000"
            if color == "rainbow":
                color = "#ff0000"  # Default to red for rainbow
            circle = plt.Circle(
                (hub.x, hub.y),
                radius=0.3,
                color=color,
                zorder=2,
            )
            self.ax.add_patch(circle)
        self._update_turn_display(self.current_turn)
        self._setup_interactive_widgets()

    def _setup_interactive_widgets(self) -> None:
        ax_next = self.fig.add_axes([0.85, 0.05, 0.1, 0.075])
        self.btn_next = plt.Button(ax_next, "Next Turn")
        self.btn_next.on_clicked(lambda event: self._on_next_turn())

    def _on_next_turn(self) -> None:
        turn = (self.current_turn + 1) % (self.result.total_turns + 1)
        self._update_turn_display(turn)
        plt.gcf().canvas.draw()
        plt.gcf().canvas.flush_events()

    def _update_turn_display(self, turn: int) -> None:
        self.current_turn = turn
        for artist in self.drone_artists:
            artist[0].remove()  # Remove scatter
            artist[1].remove()  # Remove text label
        self.drone_artists.clear()
        assert self.map_data.end_hub is not None
        end_name = self.map_data.end_hub.name
        hub_occupants: dict[str, list[int]] = {
            h: [] for h in self.map_data.hubs
        }
        transit_drones: list[tuple[int, str]] = []
        drones_at_goal = 0
        for drone_id in range(1, self.map_data.nb_drones + 1):
            loc = self.drone_locs.get(drone_id, {}).get(turn, "")
            if loc in hub_occupants:
                hub_occupants[loc].append(drone_id)
                if loc == end_name:
                    drones_at_goal += 1
            elif "-" in loc:
                transit_drones.append((drone_id, loc))
        for drone_id in range(1, self.map_data.nb_drones + 1):
            loc = self.drone_locs.get(drone_id, {}).get(turn, "")
            dx, dy = self._get_drone_coordinates(drone_id, turn, hub_occupants)
            sc = self.ax.scatter(
                dx,
                dy,
                s=130,
                color="#0000ff",
                edgecolor="#ffffff",
                linewidth=1.5,
                zorder=3,
            )
            lbl = self.ax.text(
                dx,
                dy,
                str(drone_id),
                fontsize=9,
                color="#ffffff",
                ha="center",
                va="center",
                zorder=4,
            )
            self.drone_artists.append((sc, lbl))

    def visualise(self) -> None:
        if self.fig is None:
            self.render()
        try:
            plt.show()
        except Exception as e:
            print(f"Error displaying the plot: {e}", file=sys.stderr)
