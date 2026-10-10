from dataclasses import dataclass
from collections import deque

from src2.parser import MapData
from src2.graph import TimeExpandedGraph


@dataclass
class SimulationResult:
    total_turns: int


class DroneSolver:
    def __init__(self, map_data: MapData, default_turns = 60) -> None:
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

    def solve(self, max_allowed_turns = 500) -> SimulationResult | None:
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
            if current_max_turns >= max_allowed_turns:
                break
            current_max_turns = min(current_max_turns + 30, max_allowed_turns)
        return None

    def _extract_simulation_result(
        self, graph: TimeExpandedGraph, cost: float, max_turns: int
    ) -> SimulationResult:
        return SimulationResult(total_turns=cost)