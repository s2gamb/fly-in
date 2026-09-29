import sys
import os
from typing import Optional, List
import argparse

from src2.parser import MapParser, ParsingError


class SimulationRunner:
    def __init__(self, map_path: str) -> None:
        self.map_path = map_path

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
        # debug
        print(self.map_data)
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