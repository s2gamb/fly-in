import sys
import os
from src.parser import parse_map_file


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python main.py <path_to_map_file>")
        sys.exit(1)

    map_path = sys.argv[1]

    if not os.path.exists(map_path):
        print(f"Error: File '{map_path}' not found.")
        sys.exit(1)

    parsed_map = parse_map_file(map_path)

    print(f"Parsed map: {map_path}")
    print(f"Number of drones: {parsed_map.nb_drones}")
    print(f"Start Hub: {parsed_map.start_hub.name if parsed_map.start_hub else 'None'}")
    print(f"End Hub: {parsed_map.end_hub.name if parsed_map.end_hub else 'None'}")

    print("\nHubs:")
    for hub in parsed_map.hubs.values():
        print(f"  - {hub.name} ({hub.hub_type}) at ({hub.x}, {hub.y}) | Attributes: {hub.attributes}")
    
    print("\nConnections:")
    for conn in parsed_map.connections:
        print(f"  - {conn.source} -> {conn.target} | Attributes: {conn.attributes}")

    print("\n--- Running Graph Solver (Min-Cost Max-Flow) ---")
    from src.solver import DroneSolver
    solver = DroneSolver(parsed_map, max_turns=60)
    total_turns, paths = solver.solve()

    if total_turns == -1:
        print("❌ Could not find a valid conflict-free route for all drones.")
    else:
        print(f"✅ Successfully scheduled all {parsed_map.nb_drones} drones in {total_turns} turns!\n")
        print("Drone Trajectories:")
        for drone_id, path in enumerate(paths, 1):
            route_str = " -> ".join([f"[T{t}: {hub}]" for t, hub in path])
            print(f"  Drone D{drone_id}: {route_str}")


if __name__ == "__main__":
    main()

