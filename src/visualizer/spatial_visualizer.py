"""Interactive 2D spatial graph visualizer with step-by-step turn scrolling for Fly-in."""

import math
import os
import sys
from typing import Dict, List, Optional, Set, Tuple, Union, Any

import matplotlib
# If headless environment, use non-interactive Agg backend to prevent display errors
if "DISPLAY" not in os.environ and "WAYLAND_DISPLAY" not in os.environ:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Circle
import matplotlib.lines as mlines
from matplotlib.widgets import Button, Slider
import matplotlib.animation as animation

from src.parser.models import MapData, Hub, Connection
from src.solver.drone_solver import SimulationResult
from src.visualizer.teg_visualizer import ZONE_COLORS, DRONE_PALETTE


class SpatialGraphVisualizer:
    """Renders the physical 2D graph of hubs and drones with interactive time-step scrolling."""

    def __init__(
        self,
        map_data: MapData,
        result: Optional[SimulationResult] = None,
        initial_turn: int = 0,
        figsize: Tuple[float, float] = (12.0, 9.0),
    ) -> None:
        """Initialize the spatial visualizer.

        Args:
            map_data: Parsed map network containing hubs and connections.
            result: Optional SimulationResult containing drone trajectories.
            initial_turn: Starting simulation turn to display.
            figsize: Figure dimensions (width, height).
        """
        self.map_data = map_data
        self.result = result
        self.max_turns = result.total_turns if (result and result.total_turns > 0) else 0
        self.current_turn = max(0, min(initial_turn, self.max_turns))
        self.figsize = figsize

        # Precompute drone trajectories lookup: drone_id -> Dict[turn, location_str]
        self.drone_locs: Dict[int, Dict[int, str]] = self._index_drone_locations()

        # UI & Animation elements
        self.fig: Optional[plt.Figure] = None
        self.ax: Optional[plt.Axes] = None
        self.slider: Optional[Slider] = None
        self.btn_prev: Optional[Button] = None
        self.btn_next: Optional[Button] = None
        self.btn_play: Optional[Button] = None
        self.btn_reset: Optional[Button] = None
        self.timer: Optional[Any] = None
        self.is_playing: bool = False

        # Dynamic plot artist references to update on step
        self.drone_artists: List[Any] = []
        self.movement_arrows: List[Any] = []
        self.occupancy_texts: Dict[str, Any] = {}
        self.title_text: Optional[Any] = None
        self.info_box: Optional[Any] = None

    def _index_drone_locations(self) -> Dict[int, Dict[int, str]]:
        """Build a complete turn-by-turn location mapping for every drone."""
        loc_map: Dict[int, Dict[int, str]] = {}
        start_name = self.map_data.start_hub.name if self.map_data.start_hub else "start"
        end_name = self.map_data.end_hub.name if self.map_data.end_hub else "goal"

        total_drones = self.map_data.nb_drones
        for drone_id in range(1, total_drones + 1):
            loc_map[drone_id] = {}

            if not self.result or drone_id not in self.result.paths:
                # Default: all at start hub for all turns
                for t in range(self.max_turns + 1):
                    loc_map[drone_id][t] = start_name
                continue

            traj = self.result.paths[drone_id]
            traj_dict = dict(traj)

            # Fill milestones
            curr_loc = start_name
            for t in range(self.max_turns + 1):
                if t in traj_dict:
                    curr_loc = traj_dict[t]
                loc_map[drone_id][t] = curr_loc

        return loc_map

    def _get_drone_coordinates(self, drone_id: int, turn: int, hub_occupants: Dict[str, List[int]]) -> Tuple[float, float]:
        """Compute the spatial (x, y) coordinates for a drone at a given turn."""
        loc = self.drone_locs.get(drone_id, {}).get(turn, "")
        hubs = self.map_data.hubs

        # Case 1: Drone is at a hub
        if loc in hubs:
            hub = hubs[loc]
            occupants = hub_occupants.get(loc, [drone_id])
            num_occ = len(occupants)
            if num_occ <= 1:
                return float(hub.x), float(hub.y)

            # Offset multiple occupants around the hub center
            idx = occupants.index(drone_id) if drone_id in occupants else 0
            angle = (2.0 * math.pi * idx) / num_occ
            radius = 0.22
            return hub.x + radius * math.cos(angle), hub.y + radius * math.sin(angle)

        # Case 2: Drone is in transit between two hubs: "u-v"
        if "-" in loc:
            parts = loc.split("-")
            u_name, v_name = parts[0], parts[1]
            if u_name in hubs and v_name in hubs:
                u_hub, v_hub = hubs[u_name], hubs[v_name]
                # Place in transit at midpoint
                return (u_hub.x + v_hub.x) / 2.0, (u_hub.y + v_hub.y) / 2.0

        # Fallback to start hub
        if self.map_data.start_hub:
            return float(self.map_data.start_hub.x), float(self.map_data.start_hub.y)
        return 0.0, 0.0

    def render(self) -> Tuple[plt.Figure, plt.Axes]:
        """Create the matplotlib figure, widgets, and base graph structure."""
        self.fig = plt.figure(figsize=self.figsize, dpi=100)
        self.fig.patch.set_facecolor("#ffffff")

        # Reserve space at the bottom for interactive control widgets
        self.ax = self.fig.add_axes([0.08, 0.18, 0.74, 0.74])
        self.ax.set_facecolor("#f8fafc")

        hubs = self.map_data.hubs
        xs = [h.x for h in hubs.values()]
        ys = [h.y for h in hubs.values()]
        min_x, max_x = (min(xs), max(xs)) if xs else (0, 10)
        min_y, max_y = (min(ys), max(ys)) if ys else (0, 10)
        margin_x = max(1.2, (max_x - min_x) * 0.18)
        margin_y = max(1.2, (max_y - min_y) * 0.18)

        self.ax.set_xlim(min_x - margin_x, max_x + margin_x)
        self.ax.set_ylim(min_y - margin_y, max_y + margin_y)
        self.ax.set_aspect("equal", adjustable="datalim")
        self.ax.grid(True, linestyle="--", alpha=0.4, color="#cbd5e1")
        self.ax.set_xlabel("X Coordinate", fontsize=10, fontweight="bold", labelpad=8)
        self.ax.set_ylabel("Y Coordinate", fontsize=10, fontweight="bold", labelpad=8)

        # 1. Draw Static Connections
        for conn in self.map_data.connections:
            if conn.source in hubs and conn.target in hubs:
                u = hubs[conn.source]
                v = hubs[conn.target]
                self.ax.plot(
                    [u.x, v.x], [u.y, v.y],
                    color="#94a3b8",
                    linewidth=2.0 if conn.max_link_capacity == 1 else 3.2,
                    linestyle="-",
                    zorder=1,
                )
                # Link capacity badge if > 1
                if conn.max_link_capacity > 1:
                    mid_x = (u.x + v.x) / 2.0
                    mid_y = (u.y + v.y) / 2.0
                    self.ax.text(
                        mid_x, mid_y,
                        f"cap:{conn.max_link_capacity}",
                        fontsize=7,
                        ha="center", va="center",
                        bbox=dict(boxstyle="round,pad=0.15", fc="#f1f5f9", ec="#94a3b8", lw=0.6),
                        zorder=2,
                    )

        # 2. Draw Hub Nodes
        for name, hub in hubs.items():
            color = ZONE_COLORS.get(hub.zone, ZONE_COLORS["normal"])
            if hub.hub_type == "start_hub":
                color = ZONE_COLORS["start"]
            elif hub.hub_type == "end_hub":
                color = ZONE_COLORS["end"]

            # Hub circle
            circle = plt.Circle(
                (hub.x, hub.y),
                radius=0.38,
                facecolor=color,
                edgecolor="#1e293b",
                linewidth=1.8,
                alpha=0.9,
                zorder=3,
            )
            self.ax.add_patch(circle)

            # Hub name label
            type_tag = ""
            if hub.hub_type == "start_hub":
                type_tag = " [START]"
            elif hub.hub_type == "end_hub":
                type_tag = " [GOAL]"
            elif hub.zone != "normal":
                type_tag = f" [{hub.zone.upper()}]"

            self.ax.text(
                hub.x, hub.y + 0.52,
                f"{name}{type_tag}",
                fontsize=8.5,
                fontweight="bold",
                ha="center",
                va="bottom",
                color="#0f172a",
                zorder=4,
            )

            # Dynamic Occupancy Text below hub
            occ_txt = self.ax.text(
                hub.x, hub.y - 0.55,
                "",
                fontsize=7.5,
                fontweight="medium",
                ha="center",
                va="top",
                color="#475569",
                zorder=4,
            )
            self.occupancy_texts[name] = occ_txt

        # 3. Static Legend on the right
        legend_handles = [
            mpatches.Patch(color=ZONE_COLORS["start"], label="Start Hub"),
            mpatches.Patch(color=ZONE_COLORS["end"], label="Goal Hub"),
            mpatches.Patch(color=ZONE_COLORS["priority"], label="Priority Hub"),
            mpatches.Patch(color=ZONE_COLORS["restricted"], label="Restricted Hub"),
            mpatches.Patch(color=ZONE_COLORS["normal"], label="Normal Hub"),
            mlines.Line2D([], [], color="#94a3b8", lw=2, label="Connection Link"),
            mlines.Line2D([], [], color="#2563eb", marker="o", markersize=9, markeredgecolor="white",
                          linestyle="None", label="Drone Token"),
            mlines.Line2D([], [], color="#d97706", marker="D", markersize=8, markeredgecolor="white",
                          linestyle="None", label="Drone in Transit"),
        ]
        self.fig.legend(
            handles=legend_handles,
            loc="upper right",
            bbox_to_anchor=(0.99, 0.92),
            fontsize=8.5,
            framealpha=0.95,
            edgecolor="#cbd5e1",
        )

        # 4. Turn Status Title & Movement Info Box
        self.title_text = self.fig.text(
            0.08, 0.94, "", fontsize=13, fontweight="bold", color="#0f172a"
        )
        self.info_box = self.fig.text(
            0.08, 0.91, "", fontsize=9.5, fontweight="medium", color="#334155"
        )

        # 5. Interactive Control Widgets (Buttons & Slider)
        self._setup_interactive_widgets()

        # Render initial turn state
        self._update_turn_display(self.current_turn)

        return self.fig, self.ax

    def _setup_interactive_widgets(self) -> None:
        """Create buttons and turn scrubber slider at the bottom of the figure."""
        assert self.fig is not None

        # Turn Slider: [0.15, 0.08, 0.60, 0.03]
        ax_slider = self.fig.add_axes([0.15, 0.08, 0.60, 0.03], facecolor="#f1f5f9")
        self.slider = Slider(
            ax=ax_slider,
            label="Turn: ",
            valmin=0,
            valmax=self.max_turns,
            valinit=self.current_turn,
            valstep=1,
            color="#3b82f6",
        )
        self.slider.label.set_fontsize(9)
        self.slider.label.set_fontweight("bold")
        self.slider.on_changed(self._on_slider_changed)

        # Control Buttons:
        # [Reset |<<] [Prev <] [Play/Pause] [Next >]
        btn_y = 0.025
        btn_h = 0.038

        ax_reset = self.fig.add_axes([0.18, btn_y, 0.10, btn_h])
        self.btn_reset = Button(ax_reset, "|<< Reset", color="#f8fafc", hovercolor="#e2e8f0")
        self.btn_reset.label.set_fontsize(8.5)
        self.btn_reset.on_clicked(lambda event: self.reset_turn())

        ax_prev = self.fig.add_axes([0.31, btn_y, 0.10, btn_h])
        self.btn_prev = Button(ax_prev, "< Prev", color="#f8fafc", hovercolor="#e2e8f0")
        self.btn_prev.label.set_fontsize(8.5)
        self.btn_prev.on_clicked(lambda event: self.prev_turn())

        ax_play = self.fig.add_axes([0.44, btn_y, 0.14, btn_h])
        self.btn_play = Button(ax_play, "Play ▶", color="#f8fafc", hovercolor="#e2e8f0")
        self.btn_play.label.set_fontsize(8.5)
        self.btn_play.on_clicked(lambda event: self.toggle_play())

        ax_next = self.fig.add_axes([0.61, btn_y, 0.10, btn_h])
        self.btn_next = Button(ax_next, "Next >", color="#f8fafc", hovercolor="#e2e8f0")
        self.btn_next.label.set_fontsize(8.5)
        self.btn_next.on_clicked(lambda event: self.next_turn())

        # Keyboard hotkeys
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)

    def _on_key_press(self, event: Any) -> None:
        """Handle keyboard arrow and space navigation."""
        if event.key in ("right", "d", "n"):
            self.next_turn()
        elif event.key in ("left", "a", "p"):
            self.prev_turn()
        elif event.key == " ":
            self.toggle_play()
        elif event.key in ("home", "r"):
            self.reset_turn()

    def _on_slider_changed(self, val: float) -> None:
        """Handle manual slider scrubbing."""
        new_turn = int(round(val))
        if new_turn != self.current_turn:
            self._update_turn_display(new_turn)

    def set_turn(self, turn: int) -> None:
        """Explicitly navigate to a specified turn."""
        clamped_turn = max(0, min(turn, self.max_turns))
        if self.slider and int(round(self.slider.val)) != clamped_turn:
            self.slider.set_val(clamped_turn)
        else:
            self._update_turn_display(clamped_turn)

    def next_turn(self) -> None:
        """Advance one turn forward."""
        if self.current_turn < self.max_turns:
            self.set_turn(self.current_turn + 1)
        elif self.is_playing:
            # Loop back to 0 when playback finishes
            self.set_turn(0)

    def prev_turn(self) -> None:
        """Step one turn backward."""
        if self.current_turn > 0:
            self.set_turn(self.current_turn - 1)

    def reset_turn(self) -> None:
        """Reset to initial turn 0."""
        self.set_turn(0)

    def toggle_play(self) -> None:
        """Toggle animation timer playback."""
        assert self.fig is not None
        assert self.btn_play is not None

        if self.is_playing:
            self.is_playing = False
            self.btn_play.label.set_text("Play ▶")
            if self.timer:
                self.timer.stop()
        else:
            self.is_playing = True
            self.btn_play.label.set_text("Pause ⏸")
            if not self.timer:
                self.timer = self.fig.canvas.new_timer(interval=850)
                self.timer.add_callback(self.next_turn)
            self.timer.start()

        self.fig.canvas.draw_idle()

    def _update_turn_display(self, turn: int) -> None:
        """Refresh all dynamic artists for the requested simulation turn."""
        self.current_turn = turn
        assert self.ax is not None
        assert self.fig is not None

        # 1. Clear previous turn's dynamic artists
        for artist in self.drone_artists:
            artist.remove()
        self.drone_artists.clear()

        for arrow in self.movement_arrows:
            arrow.remove()
        self.movement_arrows.clear()

        # 2. Compute hub occupancies at this turn
        hub_occupants: Dict[str, List[int]] = {h: [] for h in self.map_data.hubs}
        transit_drones: List[Tuple[int, str]] = []
        drones_at_goal = 0

        end_name = self.map_data.end_hub.name if self.map_data.end_hub else ""

        for drone_id in range(1, self.map_data.nb_drones + 1):
            loc = self.drone_locs.get(drone_id, {}).get(turn, "")
            if loc in hub_occupants:
                hub_occupants[loc].append(drone_id)
                if loc == end_name:
                    drones_at_goal += 1
            elif "-" in loc:
                transit_drones.append((drone_id, loc))

        # 3. Update Hub Occupancy Badges
        for name, occ in hub_occupants.items():
            hub = self.map_data.hubs[name]
            occ_txt = self.occupancy_texts.get(name)
            if occ_txt:
                if hub.hub_type in ("start_hub", "end_hub"):
                    badge = f"{len(occ)}/{self.map_data.nb_drones} drones"
                else:
                    badge = f"{len(occ)}/{hub.max_drones} cap"
                occ_txt.set_text(badge)
                # Highlight if occupied
                occ_txt.set_fontweight("bold" if occ else "normal")
                occ_txt.set_color("#1e293b" if occ else "#94a3b8")

        # 4. Draw Drones at Hubs
        for drone_id in range(1, self.map_data.nb_drones + 1):
            loc = self.drone_locs.get(drone_id, {}).get(turn, "")
            dx, dy = self._get_drone_coordinates(drone_id, turn, hub_occupants)
            d_color = DRONE_PALETTE[(drone_id - 1) % len(DRONE_PALETTE)]

            if "-" in loc:
                # Drone in transit: diamond marker
                sc = self.ax.scatter(
                    dx, dy,
                    s=180,
                    marker="D",
                    color=d_color,
                    edgecolors="#ffffff",
                    linewidths=1.5,
                    zorder=6,
                )
                lbl = self.ax.text(
                    dx, dy,
                    f"D{drone_id}",
                    fontsize=7,
                    fontweight="bold",
                    color="#ffffff",
                    ha="center",
                    va="center",
                    zorder=7,
                )
                self.drone_artists.extend([sc, lbl])
            else:
                # Drone at hub: circle token
                sc = self.ax.scatter(
                    dx, dy,
                    s=190,
                    marker="o",
                    color=d_color,
                    edgecolors="#ffffff",
                    linewidths=1.5,
                    zorder=6,
                )
                lbl = self.ax.text(
                    dx, dy,
                    f"D{drone_id}",
                    fontsize=7,
                    fontweight="bold",
                    color="#ffffff",
                    ha="center",
                    va="center",
                    zorder=7,
                )
                self.drone_artists.extend([sc, lbl])

        # 5. Draw Movements from turn-1 to turn
        if turn > 0:
            for drone_id in range(1, self.map_data.nb_drones + 1):
                prev_loc = self.drone_locs.get(drone_id, {}).get(turn - 1, "")
                curr_loc = self.drone_locs.get(drone_id, {}).get(turn, "")

                if prev_loc and curr_loc and prev_loc != curr_loc:
                    prev_x, prev_y = self._get_drone_coordinates(drone_id, turn - 1, {prev_loc: [drone_id]})
                    curr_x, curr_y = self._get_drone_coordinates(drone_id, turn, {curr_loc: [drone_id]})

                    d_color = DRONE_PALETTE[(drone_id - 1) % len(DRONE_PALETTE)]
                    arrow = FancyArrowPatch(
                        posA=(prev_x, prev_y),
                        posB=(curr_x, curr_y),
                        arrowstyle="-|>",
                        mutation_scale=14,
                        color=d_color,
                        linewidth=2.2,
                        linestyle="-",
                        alpha=0.85,
                        shrinkA=8,
                        shrinkB=8,
                        zorder=5,
                    )
                    self.ax.add_patch(arrow)
                    self.movement_arrows.append(arrow)

        # 6. Update Title and Info Box
        if self.title_text:
            self.title_text.set_text(
                f"Fly-in Simulation — Turn {turn} / {self.max_turns}   "
                f"({drones_at_goal}/{self.map_data.nb_drones} Drones Delivered)"
            )

        if self.info_box:
            # Chapter VII.5 formatted turn line
            turn_line = ""
            if self.result and 0 < turn <= len(self.result.turn_lines):
                turn_line = f"Turn Moves: {self.result.turn_lines[turn - 1]}"
            elif turn == 0:
                turn_line = "Turn 0: All drones ready at Start Hub."
            else:
                turn_line = "All drones arrived at goal."

            self.info_box.set_text(turn_line)

        self.fig.canvas.draw_idle()

    def save_frame(self, filepath: str, turn: Optional[int] = None, dpi: int = 200) -> str:
        """Render and save a single turn's snapshot image.

        Args:
            filepath: Destination file path (e.g. 'turn_2.png').
            turn: Turn to render (defaults to current_turn).
            dpi: Image resolution.

        Returns:
            Absolute path to saved frame.
        """
        if self.fig is None:
            self.render()

        if turn is not None:
            self._update_turn_display(turn)

        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        assert self.fig is not None
        self.fig.savefig(filepath, dpi=dpi, bbox_inches="tight")
        return os.path.abspath(filepath)

    def save_animation(self, filepath: str, fps: float = 1.0) -> str:
        """Export the full simulation playback as an animated GIF.

        Args:
            filepath: Destination .gif file path.
            fps: Playback frames per second.

        Returns:
            Absolute path to saved GIF.
        """
        if self.fig is None:
            self.render()

        def update_frame(frame: int) -> None:
            self._update_turn_display(frame)

        anim = animation.FuncAnimation(
            self.fig,
            update_frame,
            frames=self.max_turns + 1,
            repeat=True,
            repeat_delay=1500,
        )

        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        anim.save(filepath, writer="pillow", fps=fps)
        return os.path.abspath(filepath)

    def show(self) -> None:
        """Display the interactive window with buttons."""
        if self.fig is None:
            self.render()
        try:
            plt.show()
        except Exception as exc:
            print(f"Warning: Could not display interactive window: {exc}", file=sys.stderr)


def visualize_spatial_graph(
    map_data: MapData,
    result: Optional[SimulationResult] = None,
    output_path: Optional[str] = None,
    gif_path: Optional[str] = None,
    turn: int = 0,
    show: bool = False,
) -> Optional[str]:
    """Convenience helper to render and view or export the spatial graph.

    Args:
        map_data: Parsed MapData.
        result: Optional SimulationResult.
        output_path: Optional image path to save single turn snapshot.
        gif_path: Optional path to save animated GIF.
        turn: Initial turn index.
        show: If True, invoke plt.show().

    Returns:
        The output path saved if applicable.
    """
    visualizer = SpatialGraphVisualizer(
        map_data=map_data,
        result=result,
        initial_turn=turn,
    )

    saved_path = None
    if gif_path:
        saved_path = visualizer.save_animation(gif_path)
    elif output_path:
        saved_path = visualizer.save_frame(output_path, turn=turn)

    if show:
        visualizer.show()

    return saved_path
