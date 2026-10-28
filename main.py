import sys
import os
from typing import Optional, List
import argparse

from src.solver import DroneSolver, SimulationResult
from src.parser import MapParser, ParsingError, MapData
from src.visualiser import Visualiser


class SimulationRunner:
    def __init__(self, map_path: str) -> None:
        self.map_path = map_path
        self.map_data: MapData | None = None
        self.result: SimulationResult | None = None

    def _visualise_spatial_graph(self) -> None:
        if self.map_data is not None and self.result is not None:
            visualizer = Visualiser(self.map_data, self.result)
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
        solver = DroneSolver(self.map_data)
        self.result = solver.solve()
        if not self.result:
            print(
                "Error: No conflict-free path exists.",
                file=sys.stderr,
            )
            return 1
        for line in self.result.turn_lines:
            print(line)
        self._visualise_spatial_graph()
        return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fly-in: drone pathfinding and simulation tool."
    )
    parser.add_argument("map_file", help="Path to the map definition file.")
    args = parser.parse_args(argv)
    runner = SimulationRunner(map_path=args.map_file)
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
