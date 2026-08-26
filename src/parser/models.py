from dataclasses import dataclass, field


@dataclass
class Hub:
    name: str
    hub_type: str
    x: float
    y: float
    attributes: dict[str, str] = field(default_factory=dict)


@dataclass
class Connection:
    source: str
    target: str
    attributes: dict[str, str] = field(default_factory=dict)


@dataclass
class MapData:
    nb_drones: int = 0
    start_hub: Hub | None = None
    end_hub: Hub | None = None
    hubs: dict[str, Hub] = field(default_factory=dict)
    connections: list[Connection] = field(default_factory=list)
