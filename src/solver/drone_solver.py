from dataclasses import dataclass
from typing import List, Tuple, Dict
from src.parser.models import MapData
from src.graph import TimeExpandedGraph


@dataclass
class DroneStep:
    drone_id: int
    hub_name: str


class DroneSolver:
    def __init__(self, map_data: MapData, max_turns: int = 60):
        self.map_data = map_data
        self.max_turns = max_turns
        self.graph = TimeExpandedGraph(map_data, max_turns)

    def solve(self) -> Tuple[int, List[List[Tuple[int, str]]]]:
        """
        Runs Min-Cost Max-Flow and returns:
        (total_turns, list_of_drone_paths)
        where each drone path is a list of (turn, hub_name).
        """
        target_flow = self.map_data.nb_drones
        flow, cost = self.graph.network.solve_mcmf(
            self.graph.source,
            self.graph.sink,
            target_flow=target_flow
        )

        if flow < target_flow:
            # Could not route all drones within max_turns
            return -1, []

        paths: List[List[Tuple[int, str]]] = []
        end_name = self.map_data.end_hub.name

        # Extract trajectories by traversing positive flow
        for _ in range(target_flow):
            path = []
            curr = f"{self.map_data.start_hub.name}_out[0]"

            while True:
                if "_out[" in curr:
                    name = curr.split("_out[")[0]
                    turn = int(curr.split("_out[")[1].rstrip("]"))
                    path.append((turn, name))

                if curr == self.graph.sink or curr.startswith(f"{end_name}_out["):
                    break

                next_node = None
                for edge in self.graph.network.adj[curr]:
                    if edge.flow > 0:
                        edge.flow -= 1  # Trace and consume
                        next_node = edge.v
                        break

                if not next_node:
                    break
                curr = next_node

            if path:
                paths.append(path)

        # Total turns is the max turn any drone reached the destination
        max_turn_reached = 0
        for path in paths:
            if path:
                max_turn_reached = max(max_turn_reached, path[-1][0])

        return max_turn_reached, paths
