"""Main entry point for Fly-in drone simulation system."""

import sys
import os
import argparse
from typing import Optional, List

from src.parser import MapParser, MapData, ParsingError
from src.solver import DroneSolver, SimulationResult
from src.graph import TimeExpandedGraph
from src.visualizer import (
    visualize_time_expanded_graph,
    visualize_spatial_graph,
)


class SimulationRunner:
    """Encapsulates parsing, execution, and output formatting for Fly-in simulations."""

    def __init__(
        self,
        map_path: str,
        verbose: bool = False,
        stats: bool = False,
        plot_teg: bool = False,
        plot_output: Optional[str] = None,
        active_only: bool = False,
        max_turn: Optional[int] = None,
        show: bool = False,
        plot_spatial: bool = False,
        gif_output: Optional[str] = None,
        turn: int = 0,
    ) -> None:
        """Initialize the runner with CLI options.

        Args:
            map_path: Path to the input map file.
            verbose: Enable detailed diagnostics.
            stats: Enable secondary performance metrics display.
            plot_teg: Enable time-expanded graph visualization.
            plot_output: File path to save the generated visualization.
            active_only: Only display active flow/drone edges and nodes.
            max_turn: Custom maximum turn for the time horizon.
            show: Display the interactive Matplotlib window.
            plot_spatial: Enable interactive 2D spatial graph visualizer.
            gif_output: File path to save simulation playback as animated GIF.
            turn: Initial or target turn for spatial visualizer.
        """
        self.map_path = map_path
        self.verbose = verbose
        self.stats = stats
        self.plot_teg = plot_teg
        self.plot_output = plot_output
        self.active_only = active_only
        self.max_turn = max_turn
        self.show = show
        self.plot_spatial = plot_spatial
        self.gif_output = gif_output
        self.turn = turn
        self.map_data: Optional[MapData] = None
        self.result: Optional[SimulationResult] = None

    def run(self) -> int:
        """Execute the simulation pipeline.

        Returns:
            Exit code (0 on success, 1 on error).
        """
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

        if self.verbose:
            self._print_map_info()

        solver = DroneSolver(self.map_data)
        self.result = solver.solve()

        if not self.result:
            print(
                "Error: No conflict-free path exists to route all drones to the goal.",
                file=sys.stderr,
            )
            return 1

        self._print_simulation_output()

        if self.stats or self.verbose:
            self._print_statistics()

        if self.plot_teg or (self.plot_output and not self.plot_spatial and not self.gif_output):
            self._visualize_time_expanded_graph()

        if self.plot_spatial or self.gif_output:
            self._visualize_spatial_graph()

        return 0

    def _visualize_spatial_graph(self) -> None:
        """Render interactive 2D spatial graph or export simulation GIF."""
        if not self.map_data:
            return

        out_path = self.plot_output if not self.plot_teg else None
        show_interactive = self.show or (self.plot_spatial and not self.gif_output and not out_path)

        saved_path = visualize_spatial_graph(
            map_data=self.map_data,
            result=self.result,
            output_path=out_path,
            gif_path=self.gif_output,
            turn=self.turn,
            show=show_interactive,
        )

        if saved_path:
            print(f"Spatial visualization saved to: {saved_path}")

    def _visualize_time_expanded_graph(self) -> None:
        """Render and/or save the time-expanded graph visualization."""
        if not self.map_data:
            return

        graph = None
        if self.result and self.result.graph:
            graph = self.result.graph
        else:
            graph = TimeExpandedGraph(self.map_data, max_turns=self.max_turn or 15)

        output_path = self.plot_output
        if self.plot_teg and not output_path and not self.show:
            base_name = os.path.splitext(os.path.basename(self.map_path))[0]
            output_path = f"{base_name}_teg.png"

        saved_path = visualize_time_expanded_graph(
            graph=graph,
            result=self.result,
            output_path=output_path,
            max_turn=self.max_turn,
            active_only=self.active_only,
            show=self.show,
            title=f"Time-Expanded Network — {os.path.basename(self.map_path)}",
        )

        if saved_path:
            print(f"Time-expanded graph saved to: {saved_path}")

    def _print_map_info(self) -> None:
        """Print parsed map diagnostic information."""
        assert self.map_data is not None
        print(f"=== Map: {self.map_path} ===")
        print(f"Drones: {self.map_data.nb_drones}")
        start_name = self.map_data.start_hub.name if self.map_data.start_hub else "None"
        end_name = self.map_data.end_hub.name if self.map_data.end_hub else "None"
        print(f"Start Hub: {start_name} | End Hub: {end_name}")
        print(f"Hub count: {len(self.map_data.hubs)}")
        print(f"Connections count: {len(self.map_data.connections)}\n")

    def _print_simulation_output(self) -> None:
        """Print standard turn-by-turn simulation lines (Chapter VII.5)."""
        assert self.result is not None
        for line in self.result.turn_lines:
            print(line)

    def _print_statistics(self) -> None:
        """Print secondary evaluation metrics (Chapter VII.6)."""
        assert self.result is not None
        assert self.map_data is not None
        print("\n--- Simulation Metrics ---")
        print(f"Total Simulation Turns: {self.result.total_turns}")
        print(f"Average Turns per Drone: {self.result.avg_turns_per_drone}")
        print(f"Total Path Cost: {self.result.total_cost:.1f}")

        if self.verbose and self.result.paths:
            print("\nDetailed Drone Trajectories:")
            for drone_id, trajectory in self.result.paths.items():
                route = " -> ".join([f"[T{t}: {loc}]" for t, loc in trajectory])
                print(f"  D{drone_id}: {route}")


def main(argv: Optional[List[str]] = None) -> int:
    """Parse CLI arguments and run the simulation runner."""
    parser = argparse.ArgumentParser(
        description="Fly-in: High-performance multi-drone pathfinding & simulation."
    )
    parser.add_argument("map_file", help="Path to the map definition file.")
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Display verbose diagnostic logs."
    )
    parser.add_argument(
        "-s", "--stats", action="store_true", help="Display secondary simulation metrics."
    )
    parser.add_argument(
        "--plot-teg", "--visualize", action="store_true", dest="plot_teg",
        help="Visualize the time-expanded graph network."
    )
    parser.add_argument(
        "--plot-output", type=str, default=None,
        help="Save visualization plot to file path (e.g. teg.png, teg.svg)."
    )
    parser.add_argument(
        "--active-only", action="store_true",
        help="Plot only active nodes and edges traversed by drones."
    )
    parser.add_argument(
        "--max-turn", type=int, default=None,
        help="Maximum turn horizon to plot."
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display the interactive visualization window."
    )
    parser.add_argument(
        "--plot-spatial", "--visualize-spatial", action="store_true", dest="plot_spatial",
        help="Open the interactive 2D spatial visualizer with time step controls."
    )
    parser.add_argument(
        "--gif-output", type=str, default=None,
        help="Export step-by-step simulation to an animated GIF file."
    )
    parser.add_argument(
        "--turn", type=int, default=0,
        help="Initial or target turn for spatial visualization (default: 0)."
    )

    args = parser.parse_args(argv)
    runner = SimulationRunner(
        map_path=args.map_file,
        verbose=args.verbose,
        stats=args.stats,
        plot_teg=args.plot_teg,
        plot_output=args.plot_output,
        active_only=args.active_only,
        max_turn=args.max_turn,
        show=args.show,
        plot_spatial=args.plot_spatial,
        gif_output=args.gif_output,
        turn=args.turn,
    )
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())


