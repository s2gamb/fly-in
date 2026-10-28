import sys

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator


class Visualiser:
    def __init__(self, map_data, result):
        self.map_data = map_data
        self.result = result
        self.figsize = (12.0, 9.0)
        self.fig = None
        self.ax = None
        self.current_turn = 0
        self.drone_locs = self._index_drone_locations()

    def _index_drone_locations(self):
        loc_map: dict[int, dict[int, str]] = {}
        for drone_id in range(1, self.map_data.nb_drones + 1):
            loc_map[drone_id] = {}
            #TODO: handle no result case
            traj = self.result.paths[drone_id]
            traj = dict(traj)
            print(traj)

    def render(self):
        self.fig = plt.figure(figsize=self.figsize, dpi=100)
        self.ax = self.fig.add_axes([0.08, 0.18, 0.74, 0.74])
        self.ax.set_facecolor("#f8fafc")
        hubs = self.map_data.hubs
        xs = [h.x for h in hubs.values()]
        ys = [h.y for h in hubs.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        margin_x = max(1.2, (max_x - min_x) * 0.18)
        margin_y = max(1.2, (max_y - min_y) * 0.18)
        self.ax.set_xlim(min_x - margin_x, max_x + margin_x)
        self.ax.set_ylim(min_y - margin_y, max_y + margin_y)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.xaxis.set_major_locator(MultipleLocator(1))
        self.ax.yaxis.set_major_locator(MultipleLocator(1))
        self.ax.grid(True, linestyle="--", alpha=0.4, color="#cbd5e1")
        for conn in self.map_data.connections:
            hub1 = hubs[conn.source]
            hub2 = hubs[conn.target]
            self.ax.plot(
                [hub1.x, hub2.x],
                [hub1.y, hub2.y],
                color="#94a3b8",
                linewidth=3.5,
                zorder=1,
            )
        for name, hub in hubs.items():
            color = hub.color if hub.color else "#000000"
            if color == "rainbow":
                color = "#ff0000"  # Default to red for rainbow
            circle = plt.Circle(
                (hub.x, hub.y),
                radius=0.3,
                color=color,
                zorder=2,
            )
            self.ax.add_patch(circle)
        self._update_turn_display(self.current_turn)

    def _update_turn_display(self, turn: int) -> None:
        self.current_turn = turn

    def visualise(self):
        if self.fig is None:
            self.render()
        try:
            plt.show()
        except Exception as e:
            print(f"Error displaying the plot: {e}", file=sys.stderr)