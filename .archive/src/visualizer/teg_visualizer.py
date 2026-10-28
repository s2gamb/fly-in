"""Matplotlib-based visualizer for Fly-in Time-Expanded Graphs (Space-Time Network)."""

import math
import os
import sys
from collections import deque
from typing import Dict, List, Optional, Set, Tuple, Union, Any

import matplotlib
# If headless environment, use non-interactive Agg backend to prevent display errors
if "DISPLAY" not in os.environ and "WAYLAND_DISPLAY" not in os.environ:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import matplotlib.lines as mlines

from src.parser.models import MapData, Hub
from src.graph.graph_builder import TimeExpandedGraph, FlowEdge
from src.solver.drone_solver import SimulationResult


# Distinct zone color definitions
ZONE_COLORS = {
    "start": "#2ecc71",       # Emerald Green
    "end": "#e74c3c",         # Alizarin Red
    "priority": "#3498db",    # Dodger Blue
    "restricted": "#e67e22",  # Orange
    "normal": "#7f8c8d",      # Slate Gray
    "blocked": "#34495e",     # Dark Charcoal
}

# Drone trajectory color palette
DRONE_PALETTE = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
    "#2b5c8f", "#d95f02", "#7570b3", "#e7298a", "#66a61e",
]


class TimeExpandedGraphVisualizer:
    """Visualizes a Time-Expanded Graph as a 2D Space-Time Trellis network using Matplotlib."""

    def __init__(
        self,
        graph: TimeExpandedGraph,
        result: Optional[SimulationResult] = None,
        min_turn: int = 0,
        max_turn: Optional[int] = None,
        active_only: bool = False,
        highlight_drone_paths: bool = True,
        show_labels: bool = True,
        title: Optional[str] = None,
        figsize: Optional[Tuple[float, float]] = None,
    ) -> None:
        """Initialize the visualizer.

        Args:
            graph: The TimeExpandedGraph instance to visualize.
            result: Optional SimulationResult containing solved paths and metrics.
            min_turn: Lowest turn index to include on the time axis.
            max_turn: Highest turn index to include (defaults to result.total_turns or 10).
            active_only: If True, only render nodes and edges carrying flow/drones.
            highlight_drone_paths: If True, color edges by drone trajectory.
            show_labels: If True, show capacity and cost annotations on key edges.
            title: Custom plot title.
            figsize: Optional matplotlib (width, height) tuple.
        """
        self.graph = graph
        self.map_data = graph.map_data
        self.result = result
        self.min_turn = max(0, min_turn)

        # Determine sensible max_turn horizon
        if max_turn is not None:
            self.max_turn = min(max_turn, graph.max_turns)
        elif result is not None and result.total_turns > 0:
            self.max_turn = min(result.total_turns, graph.max_turns)
        else:
            self.max_turn = min(graph.max_turns, 10)

        self.active_only = active_only
        self.highlight_drone_paths = highlight_drone_paths
        self.show_labels = show_labels
        self.title = title
        self.figsize = figsize

        # Precompute hub vertical ordering and node positions
        self.hub_order: List[str] = self._order_hubs()
        self.hub_y: Dict[str, float] = {
            name: float(len(self.hub_order) - 1 - idx)
            for idx, name in enumerate(self.hub_order)
        }
        self.node_positions: Dict[str, Tuple[float, float]] = self._compute_node_positions()
        self.edge_drones: Dict[Tuple[str, str], List[int]] = self._map_drone_edges()

    def _order_hubs(self) -> List[str]:
        """Order hubs top-to-bottom topologically via BFS distance from start_hub."""
        hubs = self.map_data.hubs
        start_name = self.map_data.start_hub.name if self.map_data.start_hub else ""
        end_name = self.map_data.end_hub.name if self.map_data.end_hub else ""

        # Build adjacency for BFS
        adj: Dict[str, List[str]] = {h: [] for h in hubs}
        for conn in self.map_data.connections:
            if conn.source in hubs and conn.target in hubs:
                adj[conn.source].append(conn.target)
                adj[conn.target].append(conn.source)

        dist: Dict[str, int] = {h: 999999 for h in hubs}
        if start_name in hubs:
            dist[start_name] = 0
            q: deque[str] = deque([start_name])
            while q:
                curr = q.popleft()
                for nxt in adj.get(curr, []):
                    if dist[nxt] > dist[curr] + 1:
                        dist[nxt] = dist[curr] + 1
                        q.append(nxt)

        # Sort: start_hub first, then by BFS distance, map coordinates, and end_hub last
        def sort_key(name: str) -> Tuple[int, int, int, str]:
            if name == start_name:
                return (0, 0, 0, name)
            if name == end_name:
                return (2, 0, 0, name)
            hub = hubs[name]
            return (1, dist.get(name, 9999), hub.y, name)

        return sorted(hubs.keys(), key=sort_key)

    def _compute_node_positions(self) -> Dict[str, Tuple[float, float]]:
        """Compute (x, y) coordinates for all time-expanded graph nodes."""
        pos: Dict[str, Tuple[float, float]] = {}

        # 1. Virtual SOURCE and SINK
        start_name = self.map_data.start_hub.name if self.map_data.start_hub else ""
        end_name = self.map_data.end_hub.name if self.map_data.end_hub else ""

        start_y = self.hub_y.get(start_name, 0.0)
        end_y = self.hub_y.get(end_name, 0.0)

        pos[self.graph.source] = (self.min_turn - 0.8, start_y)
        pos[self.graph.sink] = (self.max_turn + 0.8, end_y)

        # 2. Hub in and out sub-nodes for t in [min_turn, max_turn]
        for t in range(self.min_turn, self.max_turn + 1):
            for name, y in self.hub_y.items():
                # in-node placed slightly before turn t, out-node slightly after turn t
                pos[f"{name}_in[{t}]"] = (t - 0.16, y)
                pos[f"{name}_out[{t}]"] = (t + 0.16, y)

        # 3. Intermediate transit nodes for restricted connections: transit_u->v[t]
        for node_name in self.graph.network.adj:
            if node_name.startswith("transit_") and "->" in node_name and "[" in node_name:
                # Parse transit_{u}->{v}[{t}]
                parts = node_name[len("transit_"):].split("->")
                u_name = parts[0]
                v_parts = parts[1].split("[")
                v_name = v_parts[0]
                t_val = int(v_parts[1].rstrip("]"))

                if self.min_turn <= t_val <= self.max_turn:
                    u_y = self.hub_y.get(u_name, 0.0)
                    v_y = self.hub_y.get(v_name, 0.0)
                    y_mid = (u_y + v_y) / 2.0

                    # If y_mid collides with an existing hub lane, offset it safely
                    if any(abs(y_mid - hy) < 0.28 for hy in self.hub_y.values()):
                        direction = 1.0 if u_y > v_y else -1.0
                        y_mid += 0.35 * direction

                    pos[node_name] = (float(t_val), y_mid)

        return pos

    def _map_drone_edges(self) -> Dict[Tuple[str, str], List[int]]:
        """Map directed graph edges (u, v) to the list of drone IDs that traverse them."""
        edge_drones: Dict[Tuple[str, str], List[int]] = {}
        if not self.result or not self.result.paths:
            return edge_drones

        start_name = self.map_data.start_hub.name if self.map_data.start_hub else ""
        end_name = self.map_data.end_hub.name if self.map_data.end_hub else ""

        for drone_id, trajectory in self.result.paths.items():
            if not trajectory:
                continue

            # Initial edge from SOURCE to start_out[0]
            edge_drones.setdefault((self.graph.source, f"{start_name}_out[0]"), []).append(drone_id)

            # Reconstruct edge transitions between milestones
            idx = 0
            while idx < len(trajectory) - 1:
                t1, loc1 = trajectory[idx]
                t2, loc2 = trajectory[idx + 1]

                # Case 1: Waiting at same hub loc1
                if loc1 == loc2 and t2 == t1 + 1:
                    u = f"{loc1}_out[{t1}]"
                    v = f"{loc1}_in[{t2}]"
                    edge_drones.setdefault((u, v), []).append(drone_id)

                    u_split = f"{loc1}_in[{t2}]"
                    v_split = f"{loc1}_out[{t2}]"
                    edge_drones.setdefault((u_split, v_split), []).append(drone_id)
                    idx += 1

                # Case 2: Restricted transit over 2 turns: (t, u) -> (t+1, u-v) -> (t+2, v)
                elif "-" in loc2 and t2 == t1 + 1 and idx + 2 < len(trajectory):
                    t3, loc3 = trajectory[idx + 2]
                    u_name, v_name = loc2.split("-")
                    transit_node = f"transit_{u_name}->{v_name}[{t2}]"

                    e1 = (f"{u_name}_out[{t1}]", transit_node)
                    e2 = (transit_node, f"{v_name}_in[{t3}]")
                    e3 = (f"{v_name}_in[{t3}]", f"{v_name}_out[{t3}]")

                    edge_drones.setdefault(e1, []).append(drone_id)
                    edge_drones.setdefault(e2, []).append(drone_id)
                    edge_drones.setdefault(e3, []).append(drone_id)
                    idx += 2

                # Case 3: Direct movement to neighbor hub loc2 in 1 turn
                elif loc1 != loc2 and t2 == t1 + 1:
                    u = f"{loc1}_out[{t1}]"
                    v = f"{loc2}_in[{t2}]"
                    edge_drones.setdefault((u, v), []).append(drone_id)

                    u_split = f"{loc2}_in[{t2}]"
                    v_split = f"{loc2}_out[{t2}]"
                    edge_drones.setdefault((u_split, v_split), []).append(drone_id)
                    idx += 1
                else:
                    idx += 1

            # Final edge to SINK
            final_t, final_loc = trajectory[-1]
            if final_loc == end_name:
                edge_drones.setdefault((f"{end_name}_out[{final_t}]", self.graph.sink), []).append(drone_id)

        return edge_drones

    def _get_hub_color(self, name: str) -> str:
        """Return the hex display color for a hub."""
        hub = self.map_data.hubs.get(name)
        if not hub:
            return ZONE_COLORS["normal"]

        if hub.hub_type == "start_hub":
            return ZONE_COLORS["start"]
        if hub.hub_type == "end_hub":
            return ZONE_COLORS["end"]
        if hub.zone in ZONE_COLORS:
            return ZONE_COLORS[hub.zone]
        if hub.color and hub.color in ZONE_COLORS:
            return ZONE_COLORS[hub.color]
        return ZONE_COLORS["normal"]

    def _get_edge_type(self, u: str, v: str) -> str:
        """Categorize an edge for styling."""
        if u == self.graph.source:
            return "source"
        if v == self.graph.sink:
            return "sink"
        if "_in[" in u and "_out[" in v:
            return "split"
        if "_out[" in u and "_in[" in v:
            u_hub = u.split("_out[")[0]
            v_hub = v.split("_in[")[0]
            return "wait" if u_hub == v_hub else "move"
        if "transit_" in v:
            return "transit_in"
        if "transit_" in u:
            return "transit_out"
        return "other"

    def render(self) -> Tuple[plt.Figure, plt.Axes]:
        """Render the time-expanded graph to a Matplotlib figure and axis.

        Returns:
            Tuple of (matplotlib.figure.Figure, matplotlib.axes.Axes).
        """
        # Calculate dynamic figure size if not specified
        num_turns = self.max_turn - self.min_turn + 1
        num_hubs = len(self.hub_order)
        if self.figsize:
            fig_w, fig_h = self.figsize
        else:
            fig_w = max(11.0, min(24.0, num_turns * 1.5 + 4.0))
            fig_h = max(6.0, min(18.0, num_hubs * 1.3 + 3.0))

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=100)
        fig.patch.set_facecolor("#ffffff")
        ax.set_facecolor("#fcfdfd")

        # 1. Background grid lanes for each hub
        for name in self.hub_order:
            y = self.hub_y[name]
            ax.axhline(
                y=y,
                color="#e2e8f0",
                linestyle="--",
                linewidth=0.9,
                zorder=0,
            )

        # 2. Extract and filter forward edges
        forward_edges: List[FlowEdge] = []
        for u_name, edges in self.graph.network.adj.items():
            for edge in edges:
                if edge.capacity > 0:
                    # Check if edge endpoints exist in node_positions within [min_turn, max_turn]
                    if edge.u in self.node_positions and edge.v in self.node_positions:
                        forward_edges.append(edge)

        # Active nodes and edges sets
        active_edge_keys: Set[Tuple[str, str]] = set()
        if self.result and self.result.flow_snapshot:
            for (u, v), flow in self.result.flow_snapshot.items():
                if flow > 0:
                    active_edge_keys.add((u, v))
        for (u, v), drones in self.edge_drones.items():
            if drones:
                active_edge_keys.add((u, v))

        active_nodes: Set[str] = set()
        for u, v in active_edge_keys:
            active_nodes.add(u)
            active_nodes.add(v)

        # 3. Draw Edges
        # Draw inactive edges first
        if not self.active_only:
            for edge in forward_edges:
                key = (edge.u, edge.v)
                if key in active_edge_keys:
                    continue

                p1 = self.node_positions[edge.u]
                p2 = self.node_positions[edge.v]
                etype = self._get_edge_type(edge.u, edge.v)

                # Light subtle styling for inactive edges
                edge_style = {
                    "split": ("#cbd5e1", 1.0, "solid"),
                    "wait": ("#94a3b8", 0.9, "dotted"),
                    "move": ("#cbd5e1", 1.0, "solid"),
                    "transit_in": ("#cbd5e1", 1.0, "dashed"),
                    "transit_out": ("#cbd5e1", 1.0, "dashed"),
                    "source": ("#cbd5e1", 1.2, "solid"),
                    "sink": ("#cbd5e1", 1.2, "solid"),
                }.get(etype, ("#cbd5e1", 0.9, "solid"))

                color, lw, ls = edge_style
                arrow = FancyArrowPatch(
                    posA=p1,
                    posB=p2,
                    arrowstyle="-|>",
                    mutation_scale=9,
                    color=color,
                    linestyle=ls,
                    linewidth=lw,
                    alpha=0.45,
                    shrinkA=4,
                    shrinkB=4,
                    zorder=1,
                )
                ax.add_patch(arrow)

        # Draw active edges (carrying flow / drones)
        for edge in forward_edges:
            key = (edge.u, edge.v)
            if key not in active_edge_keys:
                continue

            p1 = self.node_positions[edge.u]
            p2 = self.node_positions[edge.v]
            drones = self.edge_drones.get(key, [])
            flow = len(drones)
            if self.result and self.result.flow_snapshot and key in self.result.flow_snapshot:
                flow = max(flow, self.result.flow_snapshot[key])

            # Select color based on drone ID if available
            if self.highlight_drone_paths and drones:
                # Primary color from first drone
                d_color = DRONE_PALETTE[(drones[0] - 1) % len(DRONE_PALETTE)]
            else:
                d_color = "#2563eb"  # Royal blue for active flow

            lw = 1.8 + min(flow * 0.8, 3.5)
            etype = self._get_edge_type(edge.u, edge.v)

            conn_style = "arc3,rad=0.0"
            if etype == "sink" and (p2[0] - p1[0] > 1.2):
                # Curve long sink edges underneath the goal line
                rad = -0.15 - min(0.12, 0.04 * (p2[0] - p1[0]))
                conn_style = f"arc3,rad={rad}"

            arrow = FancyArrowPatch(
                posA=p1,
                posB=p2,
                arrowstyle="-|>",
                mutation_scale=13,
                connectionstyle=conn_style,
                color=d_color,
                linewidth=lw,
                alpha=0.9,
                shrinkA=5,
                shrinkB=5,
                zorder=3,
            )
            ax.add_patch(arrow)

            # Draw drone ID badge or capacity badge if enabled
            if self.show_labels and (drones or flow > 0):
                mid_x = (p1[0] + p2[0]) / 2.0
                mid_y = (p1[1] + p2[1]) / 2.0
                if drones:
                    badge = ",".join([f"D{d}" for d in drones])
                else:
                    badge = f"{flow}/{edge.capacity}"

                if etype == "sink" and (p2[0] - p1[0] > 1.2):
                    offset_y = -0.22
                else:
                    offset_y = 0.12 if etype in ("wait", "split") else 0.08

                ax.text(
                    mid_x,
                    mid_y + offset_y,
                    badge,
                    fontsize=7.5,
                    fontweight="bold",
                    color=d_color,
                    ha="center",
                    va="center",
                    bbox=dict(boxstyle="round,pad=0.15", fc="#ffffff", ec=d_color, lw=0.6, alpha=0.9),
                    zorder=4,
                )

        # 4. Draw Nodes
        for node_name, (nx, ny) in self.node_positions.items():
            if self.active_only and node_name not in active_nodes:
                continue

            # Special node: SOURCE
            if node_name == self.graph.source:
                ax.scatter(
                    nx, ny,
                    s=220,
                    marker="s",
                    color=ZONE_COLORS["start"],
                    edgecolors="#1b4332",
                    linewidths=1.5,
                    zorder=5,
                )
                ax.text(nx, ny, "SRC", color="white", fontsize=8, fontweight="bold", ha="center", va="center", zorder=6)
                continue

            # Special node: SINK
            if node_name == self.graph.sink:
                ax.scatter(
                    nx, ny,
                    s=220,
                    marker="s",
                    color=ZONE_COLORS["end"],
                    edgecolors="#7f1d1d",
                    linewidths=1.5,
                    zorder=5,
                )
                ax.text(nx, ny, "SNK", color="white", fontsize=8, fontweight="bold", ha="center", va="center", zorder=6)
                continue

            # Intermediate transit node
            if "transit_" in node_name:
                ax.scatter(
                    nx, ny,
                    s=120,
                    marker="D",
                    color=ZONE_COLORS["restricted"],
                    edgecolors="#7c2d12",
                    linewidths=1.2,
                    zorder=5,
                )
                continue

            # Hub sub-nodes: _in[t] and _out[t]
            if "_in[" in node_name:
                hub_name = node_name.split("_in[")[0]
                color = self._get_hub_color(hub_name)
                # In-node: open circle with zone-colored ring
                ax.scatter(
                    nx, ny,
                    s=110,
                    marker="o",
                    facecolors="#ffffff",
                    edgecolors=color,
                    linewidths=1.8,
                    zorder=5,
                )
                ax.text(nx, ny, "in", color=color, fontsize=6.5, fontweight="bold", ha="center", va="center", zorder=6)

            elif "_out[" in node_name:
                hub_name = node_name.split("_out[")[0]
                color = self._get_hub_color(hub_name)
                # Out-node: filled circle
                ax.scatter(
                    nx, ny,
                    s=110,
                    marker="o",
                    facecolors=color,
                    edgecolors="#1e293b",
                    linewidths=1.2,
                    zorder=5,
                )
                ax.text(nx, ny, "out", color="#ffffff", fontsize=6.0, fontweight="bold", ha="center", va="center", zorder=6)

        # 5. Axes, Ticks, and Labels
        ax.set_xlim(self.min_turn - 1.4, self.max_turn + 1.4)
        ax.set_ylim(-0.8, len(self.hub_order) - 0.2)

        # X-axis: Discrete Time Turns
        x_ticks = list(range(self.min_turn, self.max_turn + 1))
        ax.set_xticks(x_ticks)
        ax.set_xticklabels([f"t = {t}" for t in x_ticks], fontsize=9, fontweight="medium")
        ax.set_xlabel("Simulation Time Turn (t)", fontsize=11, fontweight="bold", labelpad=10)

        # Y-axis: Hub names with zone badges and capacities
        y_ticks = [self.hub_y[h] for h in self.hub_order]
        y_labels = []
        for name in self.hub_order:
            hub = self.map_data.hubs[name]
            zone_tag = f"[{hub.zone}]" if hub.zone != "normal" else ""
            if hub.hub_type == "start_hub":
                zone_tag = "[START]"
            elif hub.hub_type == "end_hub":
                zone_tag = "[GOAL]"

            cap_str = f"cap={hub.max_drones}" if hub.hub_type == "hub" else f"fleet={self.map_data.nb_drones}"
            y_labels.append(f"{name} {zone_tag}\n({cap_str})")

        ax.set_yticks(y_ticks)
        ax.set_yticklabels(y_labels, fontsize=9.5, fontweight="medium")
        ax.set_ylabel("Hub Locations", fontsize=11, fontweight="bold", labelpad=12)

        # 6. Title and Simulation Header
        plot_title = self.title
        if not plot_title:
            plot_title = "Time-Expanded Space-Time Network"
            if self.result:
                plot_title += f" — Solved in {self.result.total_turns} Turns (Cost: {self.result.total_cost:.1f})"

        ax.set_title(plot_title, fontsize=13, fontweight="bold", pad=15, color="#1e293b")

        # 7. Legend
        legend_handles = []

        # Node type markers
        legend_handles.append(mlines.Line2D([], [], color=ZONE_COLORS["normal"], marker="o", markerfacecolor="white",
                                            markeredgecolor=ZONE_COLORS["normal"], markersize=8, markeredgewidth=1.8,
                                            linestyle="None", label="Hub In ($u_{in}$)"))
        legend_handles.append(mlines.Line2D([], [], color="#1e293b", marker="o", markerfacecolor=ZONE_COLORS["normal"],
                                            markersize=8, markeredgewidth=1.2, linestyle="None", label="Hub Out ($u_{out}$)"))
        legend_handles.append(mlines.Line2D([], [], color="#7c2d12", marker="D", markerfacecolor=ZONE_COLORS["restricted"],
                                            markersize=8, linestyle="None", label="Transit Node"))
        legend_handles.append(mlines.Line2D([], [], color="#1b4332", marker="s", markerfacecolor=ZONE_COLORS["start"],
                                            markersize=9, linestyle="None", label="Source/Sink"))

        # Zone types
        legend_handles.append(mpatches.Patch(color=ZONE_COLORS["start"], label="Start Zone"))
        legend_handles.append(mpatches.Patch(color=ZONE_COLORS["end"], label="Goal Zone"))
        legend_handles.append(mpatches.Patch(color=ZONE_COLORS["priority"], label="Priority Zone"))
        legend_handles.append(mpatches.Patch(color=ZONE_COLORS["restricted"], label="Restricted Zone"))

        # Drone trajectories
        if self.result and self.result.paths:
            for drone_id in sorted(self.result.paths.keys())[:8]:
                d_color = DRONE_PALETTE[(drone_id - 1) % len(DRONE_PALETTE)]
                legend_handles.append(mlines.Line2D([], [], color=d_color, lw=2.5, label=f"Drone {drone_id} Path"))
            if len(self.result.paths) > 8:
                legend_handles.append(mlines.Line2D([], [], color="gray", lw=1.5, linestyle="--", label=f"+{len(self.result.paths)-8} more drones"))

        ax.legend(
            handles=legend_handles,
            bbox_to_anchor=(1.02, 1),
            loc="upper left",
            borderaxespad=0.0,
            fontsize=8.5,
            framealpha=0.95,
            edgecolor="#cbd5e1",
        )

        plt.tight_layout()
        return fig, ax

    def save(self, filepath: str, dpi: int = 300) -> str:
        """Render and save the visualization to an image file.

        Args:
            filepath: Destination file path (e.g. 'teg.png', 'teg.svg').
            dpi: Resolution dots per inch.

        Returns:
            The absolute path to the saved file.
        """
        fig, _ = self.render()
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        fig.savefig(filepath, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
        return os.path.abspath(filepath)

    def show(self) -> None:
        """Display the interactive matplotlib window."""
        self.render()
        try:
            plt.show()
        except Exception as exc:
            print(f"Warning: Could not display interactive window: {exc}", file=sys.stderr)


def visualize_time_expanded_graph(
    graph: TimeExpandedGraph,
    result: Optional[SimulationResult] = None,
    output_path: Optional[str] = None,
    max_turn: Optional[int] = None,
    active_only: bool = False,
    show: bool = False,
    title: Optional[str] = None,
) -> Optional[str]:
    """Convenience helper to visualize and optionally save a Time-Expanded Graph.

    Args:
        graph: TimeExpandedGraph instance.
        result: Optional SimulationResult.
        output_path: Optional path to save the output image.
        max_turn: Optional time horizon limit.
        active_only: If True, only render active flow/drone edges and nodes.
        show: If True, invoke plt.show().
        title: Optional custom title.

    Returns:
        The saved output path if saved, otherwise None.
    """
    visualizer = TimeExpandedGraphVisualizer(
        graph=graph,
        result=result,
        max_turn=max_turn,
        active_only=active_only,
        title=title,
    )

    saved_path = None
    if output_path:
        saved_path = visualizer.save(output_path)

    if show:
        visualizer.show()

    return saved_path
