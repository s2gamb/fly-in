"""Drone simulation solver using Min-Cost Max-Flow over a time-expanded network."""

from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Set, Optional

from src.parser.models import MapData
from src.graph import TimeExpandedGraph


@dataclass
class SimulationResult:
    """Stores the complete outcome and metrics of the drone simulation.

    Attributes:
        total_turns: Number of simulation turns required to deliver all drones.
        turn_lines: Standard formatted output string for each turn (Chapter VII.5).
        paths: Mapping from drone ID to list of (turn, location) milestones.
        total_cost: Total MCMF network cost.
        avg_turns_per_drone: Average turns taken per drone to reach goal.
    """

    total_turns: int
    turn_lines: List[str]
    paths: Dict[int, List[Tuple[int, str]]] = field(default_factory=dict)
    total_cost: float = 0.0
    avg_turns_per_drone: float = 0.0
    graph: Optional[TimeExpandedGraph] = None
    flow_snapshot: Optional[Dict[Tuple[str, str], int]] = None


class DroneSolver:
    """Coordinates pathfinding and collision-free turn scheduling for drone fleets."""

    def __init__(self, map_data: MapData, default_turns: int = 60) -> None:
        """Initialize the solver with map data.

        Args:
            map_data: Validated MapData instance.
            default_turns: Initial time-expansion horizon.
        """
        self.map_data = map_data
        self.default_turns = default_turns

    def is_connected(self) -> bool:
        """Check if a traversable path exists from start_hub to end_hub using BFS.

        Returns:
            True if end_hub is reachable from start_hub through non-blocked hubs.
        """
        if not self.map_data.start_hub or not self.map_data.end_hub:
            return False

        start_name = self.map_data.start_hub.name
        end_name = self.map_data.end_hub.name

        # Build adjacency ignoring blocked hubs
        adj: Dict[str, List[str]] = {name: [] for name in self.map_data.hubs}
        for conn in self.map_data.connections:
            u, v = conn.source, conn.target
            if (
                u in self.map_data.hubs
                and v in self.map_data.hubs
                and self.map_data.hubs[u].zone != "blocked"
                and self.map_data.hubs[v].zone != "blocked"
            ):
                adj[u].append(v)
                adj[v].append(u)

        visited: Set[str] = {start_name}
        queue: deque[str] = deque([start_name])

        while queue:
            curr = queue.popleft()
            if curr == end_name:
                return True
            for neighbor in adj.get(curr, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        return False

    def solve(self, max_allowed_turns: int = 500) -> Optional[SimulationResult]:
        """Solve drone routing minimizing total turns and avoiding conflicts.

        Args:
            max_allowed_turns: Upper limit for time horizon expansion.

        Returns:
            SimulationResult if route found, or None if routing is impossible.
        """
        if not self.is_connected():
            return None

        target_flow = self.map_data.nb_drones
        current_max_turns = max(self.default_turns, len(self.map_data.hubs) + target_flow)

        while current_max_turns <= max_allowed_turns:
            graph = TimeExpandedGraph(self.map_data, max_turns=current_max_turns)
            flow, cost = graph.network.solve_mcmf(
                graph.source,
                graph.sink,
                target_flow=target_flow,
            )

            if flow == target_flow:
                return self._extract_simulation_result(graph, cost, current_max_turns)

            # Expand search horizon if flow could not be satisfied
            if current_max_turns >= max_allowed_turns:
                break
            current_max_turns = min(current_max_turns + 30, max_allowed_turns)

        return None

    def _extract_simulation_result(
        self, graph: TimeExpandedGraph, total_cost: float, max_turns: int
    ) -> SimulationResult:
        """Trace positive flow paths and generate standard Chapter VII.5 turn output."""
        assert self.map_data.start_hub is not None
        assert self.map_data.end_hub is not None

        start_name = self.map_data.start_hub.name
        end_name = self.map_data.end_hub.name
        target_flow = self.map_data.nb_drones
        # Snapshot positive forward flows prior to path extraction
        flow_snapshot: Dict[Tuple[str, str], int] = {}
        for u_node, edges in graph.network.adj.items():
            for edge in edges:
                if edge.capacity > 0 and edge.flow > 0:
                    flow_snapshot[(edge.u, edge.v)] = edge.flow

        # turn_moves[turn] = list of formatted movement tokens: ["D1-roof1", "D2-corridorA"]
        turn_moves: Dict[int, List[Tuple[int, str]]] = {}
        drone_trajectories: Dict[int, List[Tuple[int, str]]] = {}
        delivery_turns: List[int] = []

        for drone_id in range(1, target_flow + 1):
            curr = f"{start_name}_out[0]"
            trajectory = [(0, start_name)]

            while curr != graph.sink:
                if curr.startswith(f"{end_name}_out["):
                    # Consume edge to sink
                    for edge in graph.network.adj[curr]:
                        if edge.v == graph.sink and edge.flow > 0:
                            edge.flow -= 1
                            break
                    break

                # Find outgoing edge carrying positive flow
                chosen_edge = None
                for edge in graph.network.adj[curr]:
                    if edge.flow > 0:
                        chosen_edge = edge
                        break

                if not chosen_edge:
                    break

                chosen_edge.flow -= 1
                next_node = chosen_edge.v

                if next_node == graph.sink:
                    break

                # Parse current node information: u_out[t]
                u_parts = curr.split("_out[")
                u_name = u_parts[0]
                t = int(u_parts[1].rstrip("]"))

                # Case 1: Wait in place -> next is u_in[t+1]
                if next_node == f"{u_name}_in[{t+1}]":
                    # Consume internal split edge: u_in[t+1] -> u_out[t+1]
                    for in_edge in graph.network.adj[next_node]:
                        if in_edge.v == f"{u_name}_out[{t+1}]" and in_edge.flow > 0:
                            in_edge.flow -= 1
                            break
                    curr = f"{u_name}_out[{t+1}]"
                    trajectory.append((t + 1, u_name))

                # Case 2: Multi-turn movement to restricted zone -> next is transit_u->v[t+1]
                elif next_node.startswith(f"transit_{u_name}->"):
                    v_name = next_node.split("->")[1].split("[")[0]
                    # Drone is occupying connection u-v during turn t+1
                    token = f"D{drone_id}-{u_name}-{v_name}"
                    turn_moves.setdefault(t + 1, []).append((drone_id, token))

                    # Find transit -> v_in[t+2] edge
                    for tr_edge in graph.network.adj[next_node]:
                        if tr_edge.v == f"{v_name}_in[{t+2}]" and tr_edge.flow > 0:
                            tr_edge.flow -= 1
                            break

                    # Find v_in[t+2] -> v_out[t+2] edge
                    for in_edge in graph.network.adj[f"{v_name}_in[{t+2}]"]:
                        if in_edge.v == f"{v_name}_out[{t+2}]" and in_edge.flow > 0:
                            in_edge.flow -= 1
                            break

                    # Drone arrives at v at turn t+2
                    token2 = f"D{drone_id}-{v_name}"
                    turn_moves.setdefault(t + 2, []).append((drone_id, token2))

                    curr = f"{v_name}_out[{t+2}]"
                    trajectory.append((t + 1, f"{u_name}-{v_name}"))
                    trajectory.append((t + 2, v_name))

                # Case 3: Standard movement to hub v (normal / priority) -> next is v_in[t+1]
                else:
                    v_name = next_node.split("_in[")[0]
                    # Consume internal split edge: v_in[t+1] -> v_out[t+1]
                    for in_edge in graph.network.adj[next_node]:
                        if in_edge.v == f"{v_name}_out[{t+1}]" and in_edge.flow > 0:
                            in_edge.flow -= 1
                            break

                    token = f"D{drone_id}-{v_name}"
                    turn_moves.setdefault(t + 1, []).append((drone_id, token))

                    curr = f"{v_name}_out[{t+1}]"
                    trajectory.append((t + 1, v_name))

            drone_trajectories[drone_id] = trajectory
            final_turn = trajectory[-1][0]
            delivery_turns.append(final_turn)

        max_turn = max(delivery_turns) if delivery_turns else 0

        # Construct standard turn lines (Chapter VII.5)
        formatted_turn_lines: List[str] = []
        for turn_idx in range(1, max_turn + 1):
            moves = turn_moves.get(turn_idx, [])
            if moves:
                # Sort movements by drone ID for clean deterministic output
                moves_sorted = sorted(moves, key=lambda x: x[0])
                line = " ".join([m[1] for m in moves_sorted])
                formatted_turn_lines.append(line)

        avg_turns = (sum(delivery_turns) / len(delivery_turns)) if delivery_turns else 0.0

        return SimulationResult(
            total_turns=len(formatted_turn_lines),
            turn_lines=formatted_turn_lines,
            paths=drone_trajectories,
            total_cost=total_cost,
            avg_turns_per_drone=round(avg_turns, 2),
            graph=graph,
            flow_snapshot=flow_snapshot,
        )

