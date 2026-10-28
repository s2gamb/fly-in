from dataclasses import dataclass, field
from collections import deque

from src2.parser import MapData
from src2.graph import TimeExpandedGraph


@dataclass
class SimulationResult:
    total_turns: int
    turn_lines: list[str]
    paths: dict[int, list[tuple[int, str]]] = field(default_factory=dict)
    total_cost: float = 0.0
    avg_turns_per_drone: float = (0.0,)
    graph: TimeExpandedGraph | None = None
    flow_snapshot: dict[tuple[int, str], int] | None = None


class DroneSolver:
    def __init__(self, map_data: MapData, default_turns=60) -> None:
        self.map_data = map_data
        self.default_turns = default_turns

    def _get_adjacency_list(self):
        adj = {name: [] for name in self.map_data.hubs}
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

    def solve(self, max_allowed_turns=500) -> SimulationResult | None:
        if not self.is_connected():
            return None
        target_flow = self.map_data.nb_drones
        current_max_turns = max(
            self.default_turns, len(self.map_data.hubs) + target_flow
        )
        while current_max_turns <= max_allowed_turns:
            graph = TimeExpandedGraph(
                self.map_data, max_turns=current_max_turns
            )
            flow, cost = graph.network.solve_mcmf(
                graph.source,
                graph.sink,
                target_flow=target_flow,
            )
            if flow == target_flow:
                return self._extract_simulation_result(
                    graph, cost, current_max_turns
                )
            if current_max_turns >= max_allowed_turns:
                break
            current_max_turns = min(current_max_turns + 30, max_allowed_turns)
        return None

    def _extract_simulation_result(
        self, graph: TimeExpandedGraph, total_cost: float, max_turns: int
    ) -> SimulationResult:
        start_name = self.map_data.start_hub.name
        end_name = self.map_data.end_hub.name
        target_flow = self.map_data.nb_drones
        flow_snapshot = {}
        for edges in graph.network.adj.values():
            for edge in edges:
                if edge.flow > 0:
                    flow_snapshot[(edge.u, edge.v)] = edge.flow
        turn_moves = {}
        drone_trajectories = {}
        delivery_turns = []
        for drone_id in range(1, target_flow + 1):
            curr = f"{start_name}_out[0]"
            trajectory = [(0, start_name)]
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
                if next_node == f"{source_name}_in[{t+1}]":
                    for in_edge in graph.network.adj[next_node]:
                        if (
                            in_edge.v == f"{source_name}_out[{t+1}]"
                            and in_edge.flow > 0
                        ):
                            in_edge.flow -= 1
                            break
                    curr = f"{source_name}_out[{t+1}]"
                    trajectory.append((t + 1, source_name))
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
                    for in_edge in graph.network.adj[
                        f"{target_name}_in[{t+2}]"
                    ]:
                        if (
                            in_edge.v == f"{target_name}_out[{t+2}]"
                            and in_edge.flow > 0
                        ):
                            in_edge.flow -= 1
                            break
                    token2 = f"D{drone_id}-{target_name}"
                    turn_moves.setdefault(t + 2, []).append((drone_id, token2))
                    curr = f"{target_name}_out[{t+2}]"
                    trajectory.append((t + 1, f"{source_name}-{target_name}"))
                    trajectory.append((t + 2, target_name))
                else:
                    target_name = next_node.split("_in[")[0]
                    for in_edge in graph.network.adj[next_node]:
                        if (
                            in_edge.v == f"{target_name}_out[{t+1}]"
                            and in_edge.flow > 0
                        ):
                            in_edge.flow -= 1
                            break
                    token = f"D{drone_id}-{target_name}"
                    turn_moves.setdefault(t + 1, []).append((drone_id, token))
                    curr = f"{target_name}_out[{t+1}]"
                    trajectory.append((t + 1, target_name))
            drone_trajectories[drone_id] = trajectory
            final_turn = trajectory[-1][0]
            delivery_turns.append(final_turn)
        max_turn = max(delivery_turns) if delivery_turns else 0
        formatted_turn_lines = []
        for turn_idx in range(1, max_turn + 1):
            moves = turn_moves.get(turn_idx, [])
            if moves:
                move_sorted = sorted(moves, key=lambda x: x[0])
                line = " ".join(m[1] for m in move_sorted)
                formatted_turn_lines.append(line)
        avg_turns = (
            sum(delivery_turns) / len(delivery_turns)
            if delivery_turns
            else 0.0
        )
        return SimulationResult(
            total_turns=len(formatted_turn_lines),
            turn_lines=formatted_turn_lines,
            paths=drone_trajectories,
            total_cost=total_cost,
            avg_turns_per_drone=round(avg_turns, 2),
            graph=graph,
            flow_snapshot=flow_snapshot,
        )
