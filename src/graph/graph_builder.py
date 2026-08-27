from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math
from collections import deque

from src.parser.models import MapData, Hub, Connection


@dataclass
class FlowEdge:
    u: str
    v: str
    capacity: int
    flow: int
    cost: float
    rev_index: int


class MCMFNetwork:
    """Residual flow network solved using Successive Shortest Path (SSP/SPFA)."""

    def __init__(self):
        self.adj: Dict[str, List[FlowEdge]] = {}

    def add_edge(self, u: str, v: str, capacity: int, cost: float):
        self.adj.setdefault(u, [])
        self.adj.setdefault(v, [])

        forward = FlowEdge(u, v, capacity=capacity, flow=0, cost=cost, rev_index=len(self.adj[v]))
        backward = FlowEdge(v, u, capacity=0, flow=0, cost=-cost, rev_index=len(self.adj[u]))

        self.adj[u].append(forward)
        self.adj[v].append(backward)

    def solve_mcmf(self, source: str, sink: str, target_flow: int) -> Tuple[int, float]:
        total_flow = 0
        total_cost = 0.0

        while total_flow < target_flow:
            dist: Dict[str, float] = {node: math.inf for node in self.adj}
            parent_edge: Dict[str, Optional[FlowEdge]] = {node: None for node in self.adj}
            in_queue: Dict[str, bool] = {node: False for node in self.adj}

            queue = deque([source])
            dist[source] = 0
            in_queue[source] = True

            while queue:
                u = queue.popleft()
                in_queue[u] = False

                for edge in self.adj[u]:
                    if edge.capacity - edge.flow > 0 and dist[u] + edge.cost < dist[edge.v]:
                        dist[edge.v] = dist[u] + edge.cost
                        parent_edge[edge.v] = edge
                        if not in_queue[edge.v]:
                            queue.append(edge.v)
                            in_queue[edge.v] = True

            if dist[sink] == math.inf:
                break

            curr = sink
            while curr != source:
                edge = parent_edge[curr]
                edge.flow += 1
                self.adj[edge.v][edge.rev_index].flow -= 1
                curr = edge.u

            total_flow += 1
            total_cost += dist[sink]

        return total_flow, total_cost


class TimeExpandedGraph:
    """
    Constructs a Time-Expanded Split-Node Graph from MapData enforcing:
    - Node capacity (max_drones) via (u_in -> u_out)
    - Connection capacity (max_link_capacity)
    - Restricted zones (2 turns transit with no wait)
    - Turnover rule (outgoing drones free up space on same turn)
    """

    def __init__(self, map_data: MapData, max_turns: int = 60):
        self.map_data = map_data
        self.max_turns = max_turns
        self.network = MCMFNetwork()
        self.source = "SOURCE"
        self.sink = "SINK"
        self._build()

    def _get_zone_type(self, hub: Hub) -> str:
        return hub.attributes.get("zone", hub.attributes.get("zone_type", "normal")).lower()

    def _build(self):
        hubs = self.map_data.hubs
        nb_drones = self.map_data.nb_drones

        if not self.map_data.start_hub or not self.map_data.end_hub:
            return

        start_name = self.map_data.start_hub.name

        # Connect SOURCE to start hub at t=0
        self.network.add_edge(self.source, f"{start_name}_out[0]", capacity=nb_drones, cost=0.0)

        # 1. Hub capacities and wait edges across all turns
        for t in range(self.max_turns + 1):
            for name, hub in hubs.items():
                zone = self._get_zone_type(hub)
                if zone == "blocked":
                    continue

                u_in = f"{name}_in[{t}]"
                u_out = f"{name}_out[{t}]"

                # Capacity: start and end hubs have infinite (nb_drones) capacity
                cap = nb_drones if hub.hub_type in ("start_hub", "end_hub") else int(hub.attributes.get("max_drones", 1))

                # Internal hub split edge: U_in[t] -> U_out[t]
                self.network.add_edge(u_in, u_out, capacity=cap, cost=0.0)

                # Connect end hub to sink
                if hub.hub_type == "end_hub":
                    self.network.add_edge(u_out, self.sink, capacity=nb_drones, cost=0.0)

                # Wait in place: U_out[t] -> U_in[t+1]
                if t < self.max_turns and hub.hub_type != "end_hub":
                    self.network.add_edge(u_out, f"{name}_in[{t+1}]", capacity=cap, cost=1.0)

        # 2. Connection transitions
        for conn in self.map_data.connections:
            link_cap = int(conn.attributes.get("max_link_capacity", 1))

            for u_name, v_name in [(conn.source, conn.target), (conn.target, conn.source)]:
                if u_name not in hubs or v_name not in hubs:
                    continue

                v_zone = self._get_zone_type(hubs[v_name])
                if v_zone == "blocked":
                    continue

                for t in range(self.max_turns):
                    u_out = f"{u_name}_out[{t}]"

                    if v_zone in ("normal", "priority"):
                        cost = 0.5 if v_zone == "priority" else 1.0
                        if t + 1 <= self.max_turns:
                            self.network.add_edge(u_out, f"{v_name}_in[{t+1}]", capacity=link_cap, cost=cost)

                    elif v_zone == "restricted":
                        # 2 turns required: intermediate transit node
                        if t + 2 <= self.max_turns:
                            transit = f"transit_{u_name}->{v_name}[{t+1}]"
                            self.network.add_edge(u_out, transit, capacity=link_cap, cost=1.0)
                            self.network.add_edge(transit, f"{v_name}_in[{t+2}]", capacity=link_cap, cost=1.0)
