from dataclasses import dataclass, field
from collections import deque

from src.parser import MapData
from src.graph import TimeExpandedGraph


@dataclass
class SimulationResult:
    total_turns: int
    turn_lines: list[str]
    paths: dict[int, list[tuple[int, str]]] = field(default_factory=dict)
    total_cost: float = 0.0


class DroneSolver:
    def __init__(self, map_data: MapData, default_turns: int = 10) -> None:
        self.map_data = map_data
        self.default_turns = default_turns

    def _get_adjacency_list(self) -> dict[str, list[str]]:
        adj: dict[str, list[str]] = {name: [] for name in self.map_data.hubs}
        for conn in self.map_data.connections:
            u, v = conn.source, conn.target
            if (
                self.map_data.hubs[u].zone != "blocked"
                and self.map_data.hubs[v].zone != "blocked"
            ):
                adj[u].append(v)
                adj[v].append(u)
        return adj

    def is_connected(self) -> bool:
        assert self.map_data.start_hub is not None
        assert self.map_data.end_hub is not None
        start_name = self.map_data.start_hub.name
        end_name = self.map_data.end_hub.name
        adjacency_list = self._get_adjacency_list()
        visited = set(start_name)
        queue = deque([start_name])
        while queue:
            current = queue.popleft()
            if current == end_name:
                return True
            for neighbor in adjacency_list[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        return False

    def solve(self, max_allowed_turns: int = 500) -> SimulationResult | None:
        if not self.is_connected():
            return None
        target_flow = self.map_data.nb_drones
        current_max_turns = self.default_turns
        while current_max_turns <= max_allowed_turns:
            graph = TimeExpandedGraph(
                self.map_data, max_turns=current_max_turns
            )
            flow = graph.network.solve_mcmf(
                graph.source,
                graph.sink,
                target_flow=target_flow,
            )
            if flow == target_flow:
                return self._extract_simulation_result(graph)
            if current_max_turns >= max_allowed_turns:
                break
            current_max_turns = min(current_max_turns + 30, max_allowed_turns)
        return None

    def _extract_simulation_result(
        self, graph: TimeExpandedGraph
    ) -> SimulationResult:
        assert self.map_data.start_hub is not None
        assert self.map_data.end_hub is not None
        start_name = self.map_data.start_hub.name
        end_name = self.map_data.end_hub.name
        target_flow = self.map_data.nb_drones
        turn_moves: dict[int, list[tuple[int, str]]] = {}
        drone_trajectories: dict[int, list[tuple[int, str]]] = {}
        # For each drone
        for drone_id in range(1, target_flow + 1):
            curr = f"{start_name}_out[0]"
            trajectory: list[tuple[int, str]] = [(0, start_name)]
            while curr != graph.sink:
                if curr.startswith(f"{end_name}_out["):
                    for edge in graph.network.adj[curr]:
                        if edge.v == graph.sink and edge.flow > 0:
                            edge.flow -= 1
                            break
                    break
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
                source_parts = curr.split("_out[")
                source_name = source_parts[0]
                t = int(source_parts[1].rstrip("]"))
                # Wait case
                if next_node == f"{source_name}_in[{t+1}]":
                    for edge in graph.network.adj[next_node]:
                        if (
                            edge.v == f"{source_name}_out[{t+1}]"
                            and edge.flow > 0
                        ):
                            edge.flow -= 1
                            break
                    curr = f"{source_name}_out[{t+1}]"
                    # Update trajectory
                    trajectory.append((t + 1, source_name))
                # Transit case
                elif next_node.startswith(f"transit_{source_name}->"):
                    target_name = next_node.split("->")[1].split("[")[0]
                    token = f"D{drone_id}-{source_name}-{target_name}"
                    turn_moves.setdefault(t + 1, []).append((drone_id, token))
                    for tr_edge in graph.network.adj[next_node]:
                        if (
                            tr_edge.v == f"{target_name}_in[{t+2}]"
                            and tr_edge.flow > 0
                        ):
                            tr_edge.flow -= 1
                            break
                    for edge in graph.network.adj[
                        f"{target_name}_in[{t+2}]"
                    ]:
                        if (
                            edge.v == f"{target_name}_out[{t+2}]"
                            and edge.flow > 0
                        ):
                            edge.flow -= 1
                            break
                    token2 = f"D{drone_id}-{target_name}"
                    turn_moves.setdefault(t + 2, []).append((drone_id, token2))
                    curr = f"{target_name}_out[{t+2}]"
                    # Update trajectory
                    trajectory.append((t + 1, f"{source_name}-{target_name}"))
                    trajectory.append((t + 2, target_name))
                # Default transition
                else:
                    target_name = next_node.split("_in[")[0]
                    for edge in graph.network.adj[next_node]:
                        if (
                            edge.v == f"{target_name}_out[{t+1}]"
                            and edge.flow > 0
                        ):
                            edge.flow -= 1
                            break
                    token = f"D{drone_id}-{target_name}"
                    turn_moves.setdefault(t + 1, []).append((drone_id, token))
                    curr = f"{target_name}_out[{t+1}]"
                    # Update trajectory
                    trajectory.append((t + 1, target_name))
            drone_trajectories[drone_id] = trajectory
        max_turn = max(turn_moves, default=0)
        formatted_turn_lines: list[str] = []
        for turn_idx in range(1, max_turn + 1):
            moves = turn_moves.get(turn_idx, [])
            if moves:
                move_sorted = sorted(moves, key=lambda x: x[0])
                line = " ".join(m[1] for m in move_sorted)
                formatted_turn_lines.append(line)
        return SimulationResult(
            total_turns=max_turn,
            turn_lines=formatted_turn_lines,
            paths=drone_trajectories,
        )
