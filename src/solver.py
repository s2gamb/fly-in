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

    def step(self) -> TurnSnapshot | None:
        if self.finished:
            return None

        self.current_turn += 1
        moves = " ".join(
            f"D{drone_id}-wait"
            for drone_id in sorted(self.locations)
            if self.locations[drone_id] != self.goal
        )
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
