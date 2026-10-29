import sys
import os
from typing import Optional, List
import argparse

from src.solver import DroneSimulation, TurnSnapshot
from src.parser import MapParser, ParsingError, MapData
from src.visualiser import Visualiser


class SimulationRunner:
    def __init__(self, map_path: str, seed: int | None = None) -> None:
        self.map_path = map_path
        self.seed = seed
        self.map_data: MapData | None = None
        self.simulation: DroneSimulation | None = None

    def _report_turn(self, snapshot: TurnSnapshot) -> None:
        print(
            f"Turn {snapshot.turn}: "
            f"{snapshot.moves or '(no moves)'}"
        )

    def _visualise_spatial_graph(self) -> None:
        if self.map_data is not None and self.simulation is not None:
            visualizer = Visualiser(
                self.map_data,
                self.simulation,
                on_turn=self._report_turn,
            )
            visualizer.visualise()

    def run(self) -> int:
        if not os.path.exists(self.map_path):
            print(
                f"Error: Map file '{self.map_path}' not found.",
                file=sys.stderr,
            )
            return 1
        try:
            parser = MapParser()
            self.map_data = parser.parse_file(self.map_path)
        except ParsingError as exc:
            print(str(exc), file=sys.stderr)
            return 1
        except Exception as exc:
            print(f"Unexpected error while parsing: {exc}", file=sys.stderr)
            return 1
        self.simulation = DroneSimulation(self.map_data, seed=self.seed)
        print("Click 'Next Turn' to schedule and execute the next turn.")
        self._visualise_spatial_graph()
        return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fly-in: drone pathfinding and simulation tool."
    )
    parser.add_argument("map_file", help="Path to the map definition file.")
    parser.add_argument(
        "--seed",
        type=int,
        help="Seed the random walk so a run can be reproduced.",
    )
    args = parser.parse_args(argv)
    runner = SimulationRunner(map_path=args.map_file, seed=args.seed)
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
