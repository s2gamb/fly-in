from dataclasses import dataclass
import collections

from src.parser import MapData, Connection


@dataclass
class TurnSnapshot:
    turn: int
    moves: str
    locations: dict[int, str]
    finished: bool
    completed: bool


class DroneSimulation:
    def __init__(
        self,
        map_data: MapData,
        max_turns: int = 100,
    ) -> None:
        self.map_data = map_data
        self.max_turns = max_turns
        assert map_data.start_hub is not None
        assert map_data.end_hub is not None
        self.goal = map_data.end_hub.name
        self.locations = {
            drone_id: map_data.start_hub.name
            for drone_id in range(1, map_data.nb_drones + 1)
        }
        self.current_turn = 0
        self.completed = all(
            location == self.goal for location in self.locations.values()
        )
        self.finished = self.completed

    @property
    def current_locations(self) -> dict[int, str]:
        return self.locations.copy()

    def _get_neighbors(self, hub_name: str) -> dict[str, Connection]:
        neighbors = {}
        for connection in self.map_data.connections:
            if connection.source == hub_name:
                neighbors[connection.target] = connection
            if connection.target == hub_name:
                neighbors[connection.source] = connection
        return neighbors

    def _next_hubs_toward_goal(self, hub_name: str) -> list[str]:
        distances = {self.goal: 0}
        queue = collections.deque([self.goal])
        while queue:
            current = queue.popleft()
            for neighbor in self._get_neighbors(current):
                if neighbor not in distances:
                    distances[neighbor] = distances[current] + 1
                    queue.append(neighbor)
        if hub_name not in distances:
            raise ValueError(f"No route from hub '{hub_name}' to '{self.goal}'.")
        return [
            neighbor
            for neighbor in self._get_neighbors(hub_name)
            if distances.get(neighbor) == distances[hub_name] - 1
        ]

    def _reset_connection_capacities(self):
        for connection in self.map_data.connections:
            connection.capacity = 0

    def _get_hub(self, hub_name):
        return self.map_data.hubs[hub_name]

    def step(self) -> TurnSnapshot | None:
        if self.finished:
            return None
        self.current_turn += 1
        next_locations = self.current_locations
        moves: list[str] = []
        for drone_id in range(1, self.map_data.nb_drones + 1):
            location = self.locations[drone_id]
            if location == self.goal:
                continue
            neighbor_locations: list[str] = self._next_hubs_toward_goal(location)
            neighbors = self._get_neighbors(location)
            next_location = location
            for next_hub in neighbor_locations:
                prev_hub = self._get_hub(location)
                hub = self._get_hub(next_hub)
                if neighbors[next_hub].capacity < neighbors[next_hub].max_link_capacity and hub.capacity < hub.max_drones:
                    hub.capacity += 1
                    prev_hub.capacity -= 1
                    neighbors[next_hub].capacity += 1
                    next_location = next_hub
                    next_locations[drone_id] = next_location
                    break
            if next_location != location:
                moves.append(f"D{drone_id}-{next_location}")
        self.locations = next_locations
        self._reset_connection_capacities()
        self.completed = all(
            location == self.goal for location in self.locations.values()
        )
        self.finished = (
            self.completed or self.current_turn >= self.max_turns
        )
        return TurnSnapshot(
            turn=self.current_turn,
            moves=" ".join(moves),
            locations=self.current_locations,
            finished=self.finished,
            completed=self.completed,
        )
