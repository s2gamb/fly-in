import math
from collections import deque

from dataclasses import dataclass
from src.parser import MapData, Hub


@dataclass
class FlowEdge:
    u: str
    v: str
    capacity: int
    flow: int
    cost: float
    rev_index: int


class MCMFNetwork:
    def __init__(self) -> None:
        # Network consist of the list of adjacent edges from each node
        self.adj: dict[str, list[FlowEdge]] = {}

    def add_edge(self, u: str, v: str, capacity: int, cost: float) -> None:
        self.adj.setdefault(u, [])
        self.adj.setdefault(v, [])
        forward = FlowEdge(
            u=u,
            v=v,
            # Initially full capacity
            capacity=capacity,
            flow=0,
            cost=cost,
            rev_index=len(self.adj[v]),
        )
        backward = FlowEdge(
            u=v,
            v=u,
            # Empty capacity for backward edge
            capacity=0,
            flow=0,
            cost=-cost,
            rev_index=len(self.adj[u]),
        )
        self.adj[u].append(forward)
        self.adj[v].append(backward)

    def solve_mcmf(
        self, source: str, sink: str, target_flow: int
    ) -> int:
        total_flow = 0
        # Iterating by flow one by one
        while total_flow < target_flow:
            dist: dict[str, float] = {node: math.inf for node in self.adj}
            parent_edge: dict[str, FlowEdge | None] = {
                node: None for node in self.adj
            }
            in_queue: dict[str, bool] = {node: False for node in self.adj}
            queue = deque([source])
            dist[source] = 0.0
            in_queue[source] = True
            while queue:
                u = queue.popleft()
                in_queue[u] = False
                # Traverse adjacents nodes
                for edge in self.adj[u]:
                    # If the capacity allows and the distance optimal
                    if (
                        edge.capacity - edge.flow > 0
                        and dist[u] + edge.cost < dist[edge.v]
                    ):
                        # Update distance
                        dist[edge.v] = dist[u] + edge.cost
                        parent_edge[edge.v] = edge
                        if not in_queue[edge.v]:
                            queue.append(edge.v)
                            in_queue[edge.v] = True
            if dist[sink] == math.inf:
                # Can not reach sink
                break
            curr = sink
            # Traverse backward
            while curr != source:
                p_edge = parent_edge[curr]
                assert p_edge is not None
                # Update edge flow
                p_edge.flow += 1
                # Update reverse edge flow
                self.adj[p_edge.v][p_edge.rev_index].flow -= 1
                curr = p_edge.u
            # Successfully transfered one drone to sink
            total_flow += 1
        return total_flow


class TimeExpandedGraph:
    def __init__(self, map_data: MapData, max_turns: int) -> None:
        self.map_data = map_data
        self.max_turns = max_turns
        self.network = MCMFNetwork()
        self.source = "SOURCE"
        self.sink = "SINK"
        self._build()

    def _get_zone_type(self, hub: Hub) -> str:
        return hub.zone

    def _build(self) -> None:
        hubs = self.map_data.hubs
        nb_drones = self.map_data.nb_drones
        assert self.map_data.start_hub is not None
        start_name = self.map_data.start_hub.name
        # Always can move from SOURCE to start hub out on turn zero
        self.network.add_edge(
            self.source, f"{start_name}_out[0]", capacity=nb_drones, cost=0.0
        )
        for t in range(self.max_turns + 1):
            for name, hub in hubs.items():
                zone = self._get_zone_type(hub)
                if zone == "blocked":
                    continue
                # Create new nodes for each hub for each turn
                u_in = f"{name}_in[{t}]"
                u_out = f"{name}_out[{t}]"
                cap = (
                    nb_drones
                    if hub.hub_type in ("start_hub", "end_hub")
                    else hub.max_drones
                )
                # We always can move from hub_in to hub_out
                self.network.add_edge(u_in, u_out, capacity=cap, cost=0.0)
                if hub.hub_type == "end_hub":
                    self.network.add_edge(
                        # We always can move to SINK node
                        u_out, self.sink, capacity=nb_drones, cost=0.0
                    )
                if t < self.max_turns and hub.hub_type != "end_hub":
                    # And we can wait on that hub
                    self.network.add_edge(
                        u_out, f"{name}_in[{t+1}]", capacity=cap, cost=1.0
                    )
        for conn in self.map_data.connections:
            link_cap = conn.max_link_capacity
            for u_name, v_name in [
                (conn.source, conn.target),
                (conn.target, conn.source),
            ]:
                v_zone = self._get_zone_type(hubs[v_name])
                if v_zone == "blocked":
                    continue
                for t in range(self.max_turns):
                    u_out = f"{u_name}_out[{t}]"
                    if v_zone in ("normal", "priority"):
                        cost = 0.5 if v_zone == "priority" else 1.0
                        if t + 1 <= self.max_turns:
                            # Default transition from hub to hub
                            self.network.add_edge(
                                u_out,
                                f"{v_name}_in[{t+1}]",
                                capacity=link_cap,
                                cost=cost,
                            )
                    elif v_zone == "restricted":
                        if t + 2 <= self.max_turns:
                            transit = f"transit_{u_name}->{v_name}[{t+1}]"
                            # Transition via transit node cost 2 turns
                            self.network.add_edge(
                                u_out, transit, capacity=link_cap, cost=1.0
                            )
                            self.network.add_edge(
                                transit,
                                f"{v_name}_in[{t+2}]",
                                capacity=link_cap,
                                cost=1.0,
                            )
