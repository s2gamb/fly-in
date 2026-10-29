import math
import sys
from typing import Any, Callable

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backend_bases import MouseEvent
from matplotlib.colors import to_rgb
from matplotlib.ticker import MultipleLocator

from src.parser import MapData
from src.solver import DroneSimulation, TurnSnapshot


class Visualiser:
    def __init__(
        self,
        map_data: MapData,
        simulation: DroneSimulation,
        on_turn: Callable[[TurnSnapshot], None] | None = None,
    ) -> None:
        self.map_data = map_data
        self.simulation = simulation
        self.on_turn = on_turn
        self.figsize: tuple[float, float] = (12.0, 9.0)
        self.fig: Any = None
        self.ax: Any = None
        self.current_turn = simulation.current_turn
        self.current_locations = simulation.current_locations
        self.btn_next: Any = None
        self.drone_artists: list[tuple[Any, Any]] = []
        self.animation: FuncAnimation | None = None
        self.is_animating = False
        self.animation_frames = 5

    def _coordinates_for_locations(
        self, locations: dict[int, str]
    ) -> dict[int, tuple[float, float]]:
        hubs = self.map_data.hubs
        occupants: dict[str, list[int]] = {
            name: [] for name in hubs
        }
        for drone_id, location in locations.items():
            if location in occupants:
                occupants[location].append(drone_id)

        coordinates: dict[int, tuple[float, float]] = {}
        for drone_id, location in locations.items():
            if location in hubs:
                hub = hubs[location]
                hub_occupants = occupants[location]
                if len(hub_occupants) <= 1:
                    coordinates[drone_id] = (
                        float(hub.x),
                        float(hub.y),
                    )
                else:
                    index = hub_occupants.index(drone_id)
                    angle = (2.0 * math.pi * index) / len(hub_occupants)
                    coordinates[drone_id] = (
                        float(hub.x) + 0.16 * math.cos(angle),
                        float(hub.y) + 0.16 * math.sin(angle),
                    )
            elif "-" in location:
                source, target = location.split("-", 1)
                if source in hubs and target in hubs:
                    source_hub = hubs[source]
                    target_hub = hubs[target]
                    coordinates[drone_id] = (
                        (float(source_hub.x) + float(target_hub.x)) / 2.0,
                        (float(source_hub.y) + float(target_hub.y)) / 2.0,
                    )
                else:
                    coordinates[drone_id] = (0.0, 0.0)
            else:
                coordinates[drone_id] = (0.0, 0.0)
        return coordinates

    @staticmethod
    def _pastel_color(color: str) -> tuple[float, float, float]:
        rgb = to_rgb("#ff0000" if color == "rainbow" else color)
        return tuple(channel * 0.55 + 0.45 for channel in rgb)

    def _set_drone_positions(
        self, positions: dict[int, tuple[float, float]]
    ) -> None:
        for drone_id, (scatter, label) in enumerate(
            self.drone_artists, start=1
        ):
            position = positions[drone_id]
            scatter.set_offsets([position])
            label.set_position(position)

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
            if conn.max_link_capacity != 1:
                self.ax.text(
                    (hub1.x + hub2.x) / 2.0,
                    (hub1.y + hub2.y) / 2.0,
                    f"cap {conn.max_link_capacity}",
                    fontsize=8,
                    color="#334155",
                    ha="center",
                    va="center",
                    bbox={
                        "boxstyle": "round,pad=0.2",
                        "facecolor": "#ffffff",
                        "edgecolor": "#94a3b8",
                        "linewidth": 0.8,
                    },
                    zorder=4,
                )
        for hub in hubs.values():
            color = self._pastel_color(
                hub.color if hub.color else "#000000"
            )
            circle = plt.Circle(
                (hub.x, hub.y),
                radius=0.3,
                color=color,
                zorder=2,
            )
            self.ax.add_patch(circle)
            if (
                hub.hub_type not in ("start_hub", "end_hub")
                and hub.max_drones != 1
            ):
                self.ax.text(
                    hub.x,
                    hub.y - 0.43,
                    f"cap {hub.max_drones}",
                    fontsize=7,
                    color="#334155",
                    ha="center",
                    va="center",
                    bbox={
                        "boxstyle": "round,pad=0.15",
                        "facecolor": "#ffffff",
                        "edgecolor": "#cbd5e1",
                        "linewidth": 0.7,
                    },
                    zorder=4,
                )
            if hub.zone == "restricted":
                marker = plt.Circle(
                    (hub.x, hub.y),
                    radius=0.42,
                    fill=False,
                    edgecolor="#7c3aed",
                    linewidth=1.5,
                    linestyle=(0, (3, 2)),
                    zorder=2.5,
                )
                self.ax.add_patch(marker)
                self.ax.text(
                    hub.x + 0.25,
                    hub.y + 0.25,
                    "R",
                    color="#6d28d9",
                    fontsize=7,
                    fontweight="bold",
                    ha="center",
                    va="center",
                    bbox={
                        "boxstyle": "circle,pad=0.2",
                        "facecolor": "#ffffff",
                        "edgecolor": "#7c3aed",
                        "linewidth": 0.8,
                    },
                    zorder=5,
                )
        self._update_turn_display(
            self.current_turn, self.current_locations
        )
        self._setup_interactive_widgets()

    def _setup_interactive_widgets(self) -> None:
        ax_next = self.fig.add_axes([0.85, 0.05, 0.1, 0.075])
        self.btn_next = plt.Button(ax_next, "Next Turn")
        self.btn_next.on_clicked(self._on_next_turn)

    def _on_next_turn(self, event: MouseEvent) -> None:
        if self.simulation.finished or self.is_animating:
            return

        previous_locations = self.current_locations.copy()
        start_positions = self._coordinates_for_locations(previous_locations)
        snapshot = self.simulation.step()
        if snapshot is None:
            return
        if self.on_turn is not None:
            self.on_turn(snapshot)
        self.current_turn = snapshot.turn
        self.current_locations = snapshot.locations
        end_positions = self._coordinates_for_locations(
            self.current_locations
        )
        self.is_animating = True
        self.btn_next.set_active(False)

        def animate(frame: int) -> tuple[Any, ...]:
            progress = frame / self.animation_frames
            progress = progress * progress * (3.0 - 2.0 * progress)
            positions = {
                drone_id: (
                    start_positions[drone_id][0]
                    + progress
                    * (
                        end_positions[drone_id][0]
                        - start_positions[drone_id][0]
                    ),
                    start_positions[drone_id][1]
                    + progress
                    * (
                        end_positions[drone_id][1]
                        - start_positions[drone_id][1]
                    ),
                )
                for drone_id in start_positions
            }
            self._set_drone_positions(positions)
            return tuple(
                artist
                for pair in self.drone_artists
                for artist in pair
            )

        def finish_animation() -> None:
            self.is_animating = False
            if snapshot.finished:
                label = "Complete" if snapshot.completed else "Stopped"
                self.btn_next.label.set_text(label)
            else:
                self.btn_next.set_active(True)

        def animate_and_finish(frame: int) -> tuple[Any, ...]:
            artists = animate(frame)
            self.ax.set_title(f"Turn {snapshot.turn}")
            if frame == self.animation_frames:
                finish_animation()
            return artists

        event.canvas.draw_idle()
        self.animation = FuncAnimation(
            self.fig,
            animate_and_finish,
            frames=self.animation_frames + 1,
            interval=60,
            repeat=False,
            blit=False,
        )

    def _update_turn_display(
        self, turn: int, locations: dict[int, str]
    ) -> None:
        self.current_turn = turn
        self.current_locations = locations
        positions = self._coordinates_for_locations(locations)
        for drone_id in range(1, self.map_data.nb_drones + 1):
            dx, dy = positions[drone_id]
            scatter = self.ax.scatter(
                dx,
                dy,
                s=130,
                color="#0000ff",
                edgecolor="#ffffff",
                linewidth=1.5,
                zorder=3,
            )
            label = self.ax.text(
                dx,
                dy,
                str(drone_id),
                fontsize=9,
                color="#ffffff",
                ha="center",
                va="center",
                zorder=4,
            )
            self.drone_artists.append((scatter, label))
        self.ax.set_title(f"Turn {turn}")

    def visualise(self) -> None:
        if self.fig is None:
            self.render()
        try:
            plt.show()
        except Exception as e:
            print(f"Error displaying the plot: {e}", file=sys.stderr)
