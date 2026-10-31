from dataclasses import dataclass
import collections
import time

from src.parser import MapData


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
        self._next_hub_toward_goal(self.map_data.start_hub.name)

    @property
    def current_locations(self) -> dict[int, str]:
        return self.locations.copy()

    def _get_neighbors(self, hub_name: str) -> list[str]:
        neighbors = []
        for connection in self.map_data.connections:
            if connection.source == hub_name:
                neighbors.append(connection.target)
            if connection.target == hub_name:
                neighbors.append(connection.source)
        return neighbors

    def _next_hub_toward_goal(self, hub_name: str) -> str:
        queue = collections.deque([hub_name])
        in_queue = {hub: False for hub in self.map_data.hubs}
        in_queue[hub_name] = True
        prev = {hub: None for hub in self.map_data.hubs}
        while queue:
            print(queue)
            current = queue.popleft()
            if current == self.map_data.end_hub.name:
                break
            neighbors = self._get_neighbors(current)
            for neighbor in neighbors:
                if not in_queue[neighbor]:
                    queue.append(neighbor)
                    in_queue[neighbor] = True
                    prev[neighbor] = current
            time.sleep(1)
        if current == self.map_data.end_hub.name:
            print("Found end hub")
        # Go back
        while prev[current] != self.map_data.start_hub.name:
            current = prev[current]
        return current

    def step(self) -> TurnSnapshot | None:
        if self.finished:
            return None
        self.current_turn += 1
        moves = None
        self.finished = (
            self.completed or self.current_turn >= self.max_turns
        )
        return TurnSnapshot(
            turn=self.current_turn,
            moves=moves,
            locations=self.current_locations,
            finished=self.finished,
            completed=self.completed,
        )
