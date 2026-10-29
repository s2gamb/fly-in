import random
from collections import defaultdict
from dataclasses import dataclass

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
        seed: int | None = None,
        max_turns: int = 100,
    ) -> None:
        self.map_data = map_data
        self.random = random.Random(seed)
        self.max_turns = max_turns
        self.adjacency = self._get_adjacency_list()
        self.connection_capacities = {
            self._connection_key(conn.source, conn.target):
                conn.max_link_capacity
            for conn in map_data.connections
        }
        assert map_data.start_hub is not None
        assert map_data.end_hub is not None
        self.goal = map_data.end_hub.name
        self.states: dict[int, str | tuple[str, str]] = {
            drone_id: map_data.start_hub.name
            for drone_id in range(1, map_data.nb_drones + 1)
        }
        self.current_turn = 0
        self.finished = False
        self.completed = all(
            location == self.goal for location in self.states.values()
        )
        self.finished = self.completed

    @staticmethod
    def _connection_key(source: str, target: str) -> tuple[str, str]:
        return min(source, target), max(source, target)

    @staticmethod
    def _display_location(location: str | tuple[str, str]) -> str:
        if isinstance(location, str):
            return location
        source, target = location
        return f"{source}-{target}"

    @property
    def current_locations(self) -> dict[int, str]:
        return {
            drone_id: self._display_location(location)
            for drone_id, location in self.states.items()
        }

    def _get_adjacency_list(self) -> dict[str, list[str]]:
        adjacency = {name: [] for name in self.map_data.hubs}
        for connection in self.map_data.connections:
            if (
                self.map_data.hubs[connection.source].zone == "blocked"
                or self.map_data.hubs[connection.target].zone == "blocked"
            ):
                continue
            adjacency[connection.source].append(connection.target)
            adjacency[connection.target].append(connection.source)
        return adjacency

    def _hub_capacity(self, name: str) -> int:
        hub = self.map_data.hubs[name]
        if hub.hub_type in ("start_hub", "end_hub"):
            return self.map_data.nb_drones
        return hub.max_drones

    def step(self) -> TurnSnapshot | None:
        if self.finished:
            return None

        turn = self.current_turn + 1
        drone_ids = list(self.states)
        offset = (turn - 1) % len(drone_ids)
        turn_order = drone_ids[offset:] + drone_ids[:offset]

        occupancy = defaultdict(int)
        for location in self.states.values():
            if isinstance(location, str):
                occupancy[location] += 1
        drones_at_hubs = {
            drone_id
            for drone_id, location in self.states.items()
            if isinstance(location, str)
        }
        connection_usage = defaultdict(int)
        moves: list[tuple[int, str]] = []

        for drone_id in turn_order:
            location = self.states[drone_id]
            if isinstance(location, str):
                continue
            source, target = location
            connection = self._connection_key(source, target)
            if (
                connection_usage[connection]
                >= self.connection_capacities[connection]
                or occupancy[target] >= self._hub_capacity(target)
            ):
                continue
            connection_usage[connection] += 1
            occupancy[target] += 1
            self.states[drone_id] = target
            moves.append((drone_id, f"D{drone_id}-{target}"))

        for drone_id in turn_order:
            location = self.states[drone_id]
            if (
                drone_id not in drones_at_hubs
                or not isinstance(location, str)
                or location == self.goal
            ):
                continue
            candidates = [
                neighbor
                for neighbor in self.adjacency[location]
                if neighbor == self.goal
                or occupancy[neighbor] < self._hub_capacity(neighbor)
            ]
            self.random.shuffle(candidates)

            for target in candidates:
                connection = self._connection_key(location, target)
                if (
                    connection_usage[connection]
                    >= self.connection_capacities[connection]
                ):
                    continue

                connection_usage[connection] += 1
                occupancy[location] -= 1
                if self.map_data.hubs[target].zone == "restricted":
                    self.states[drone_id] = (location, target)
                    moves.append(
                        (drone_id, f"D{drone_id}-{location}-{target}")
                    )
                else:
                    occupancy[target] += 1
                    self.states[drone_id] = target
                    moves.append((drone_id, f"D{drone_id}-{target}"))
                break

        self.current_turn = turn
        turn_moves = " ".join(move for _, move in sorted(moves))
        self.completed = all(
            location == self.goal for location in self.states.values()
        )
        self.finished = (
            self.completed or not moves or turn >= self.max_turns
        )

        return TurnSnapshot(
            turn=turn,
            moves=turn_moves,
            locations=self.current_locations,
            finished=self.finished,
            completed=self.completed,
        )
