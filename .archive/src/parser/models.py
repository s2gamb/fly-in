"""Data models and exception definitions for map parsing."""

from dataclasses import dataclass, field
from typing import Optional, Dict, List


class ParsingError(Exception):
    """Exception raised for syntax or semantic errors in map files."""

    def __init__(self, message: str, line_num: Optional[int] = None) -> None:
        """Initialize ParsingError with an error message and optional line number.

        Args:
            message: Description of the error.
            line_num: 1-indexed line number where the error occurred.
        """
        super().__init__(message)
        self.message = message
        self.line_num = line_num

    def __str__(self) -> str:
        """Format the parsing error with line number if available."""
        if self.line_num is not None:
            return f"Parsing error on line {self.line_num}: {self.message}"
        return f"Parsing error: {self.message}"


@dataclass
class Hub:
    """Represents a zone/hub in the drone network.

    Attributes:
        name: Unique identifier of the hub (no dashes or whitespace).
        hub_type: Prefix type ('start_hub', 'end_hub', or 'hub').
        x: Integer X coordinate.
        y: Integer Y coordinate.
        zone: Zone type ('normal', 'blocked', 'restricted', 'priority').
        color: Optional display color string.
        max_drones: Maximum concurrent drone capacity (ignored on start/end).
        attributes: Raw dictionary of parsed attributes.
    """

    name: str
    hub_type: str
    x: int
    y: int
    zone: str = "normal"
    color: Optional[str] = None
    max_drones: int = 1
    attributes: Dict[str, str] = field(default_factory=dict)


@dataclass
class Connection:
    """Represents a bidirectional connection between two hubs.

    Attributes:
        source: Name of the origin hub.
        target: Name of the destination hub.
        max_link_capacity: Maximum simultaneous drones allowed on the connection.
        attributes: Raw dictionary of parsed attributes.
    """

    source: str
    target: str
    max_link_capacity: int = 1
    attributes: Dict[str, str] = field(default_factory=dict)


@dataclass
class MapData:
    """Encapsulates the full network configuration and fleet size.

    Attributes:
        nb_drones: Number of drones to route from start to end.
        start_hub: Starting zone hub.
        end_hub: Target destination hub.
        hubs: Mapping of hub name to Hub instance.
        connections: List of unique bidirectional connections.
    """

    nb_drones: int = 0
    start_hub: Optional[Hub] = None
    end_hub: Optional[Hub] = None
    hubs: Dict[str, Hub] = field(default_factory=dict)
    connections: List[Connection] = field(default_factory=list)
