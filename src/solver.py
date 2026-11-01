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

    def _get_neighbors(self, hub_name: str) -> list[tuple[str, Connection]]:
        neighbors = {}
        for connection in self.map_data.connections:
            if connection.source == hub_name:
                neighbors[connection.target] = connection
            if connection.target == hub_name:
                neighbors[connection.source] = connection
        return neighbors

    def _next_hub_toward_goal(self, hub_name: str) -> str:
        if hub_name == self.goal:
            return hub_name
        queue = collections.deque([hub_name])
        previous = {}
        while queue:
            current = queue.popleft()
            if current == self.goal:
                break
            neighbors = self._get_neighbors(current)
            for neighbor, connection in neighbors.items():
                if neighbor not in previous:
                    queue.append(neighbor)
                    previous[neighbor] = current
        next_hub = self.goal
        while previous[next_hub] != hub_name:
            next_hub = previous[next_hub]
        neighbors = self._get_neighbors(hub_name)
        if neighbors[next_hub].capacity < neighbors[next_hub].max_link_capacity:
            neighbors[next_hub].capacity += 1
            return next_hub
        return hub_name

    def _reset_connection_capacities(self):
        for connection in self.map_data.connections:
            connection.capacity = 0

    def step(self) -> TurnSnapshot | None:
        if self.finished:
            return None
        self.current_turn += 1
        next_locations = self.current_locations
        moves: list[str] = []
        for drone_id in range(1, self.map_data.nb_drones + 1):
            location = self.locations[drone_id]
            next_location = self._next_hub_toward_goal(location)
            next_locations[drone_id] = next_location
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
