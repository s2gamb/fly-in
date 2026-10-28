import sys
import os
from typing import Optional, List
import argparse

from src2.solver import DroneSolver
from src2.parser import MapParser, ParsingError
from src2.visualiser import Visualiser


class SimulationRunner:
    def __init__(self, map_path: str) -> None:
        self.map_path = map_path

    def _print_simulation_output(self) -> None:
        for line in self.result.turn_lines:
            print(line)

    def _visualise_spatial_graph(self) -> None:
        visualizer = Visualiser(self.map_data, self.result)
        visualizer.visualise()

    def run(self) -> None:
        if not os.path.exists(self.map_path):
            print(f"Error: Map file '{self.map_path}' not found.", file=sys.stderr)
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
                "Error: No conflict-free path exists to route all drones to the goal.",
                file=sys.stderr,
            )
            return 1
        self._print_simulation_output()
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