import re
from dataclasses import dataclass, field


ALLOWED_ZONE_TYPES: set[str] = {"normal", "blocked", "restricted", "priority"}


class ParsingError(Exception):
    def __init__(self, message: str, line_num: int | None = None) -> None:
        super().__init__(message)
        self.message = message
        self.line_num = line_num

    def __str__(self) -> str:
        if self.line_num is not None:
            return f"Parsing error on line {self.line_num}: {self.message}"
        return f"Parsing error: {self.message}"


@dataclass
class Hub:
    name: str
    hub_type: str
    x: int
    y: int
    zone: str = "normal"
    color: str | None = None
    max_drones: int = 1

@dataclass
class Connection:
    source: str
    target: str
    max_link_capacity: int = 1

@dataclass
class MapData:
    nb_drones: int = 0
    hubs: dict[str, Hub] = field(default_factory=dict)
    start_hub: Hub | None = None
    end_hub: Hub | None = None
    connections: list[Connection] = field(default_factory=list)

    def __str__(self) -> str:
        return (
            f"MapData(nb_drones={self.nb_drones}, "
            f"hubs={list(self.hubs.keys())}, "
            f"start_hub={self.start_hub.name if self.start_hub else None}, "
            f"end_hub={self.end_hub.name if self.end_hub else None}, "
            f"connections={[f'{c.source}-{c.target}[{c.max_link_capacity}]' for c in self.connections]})"
        )

class MapParser:
    def __init__(self):
        self.map_data: MapData = MapData()
        self.seen_connections: set[tuple[str, str]] = set()

    def parse_file(self, file_path: str) -> MapData:
        with open(file_path, "r") as f:
            return self.parse_text(f.read())

    def parse_text(self, text: str) -> MapData:
        lines = text.splitlines()
        first_non_empty_seen = False
        for line_num, line in enumerate(lines, start=1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            if not first_non_empty_seen:
                if not line.startswith("nb_drones"):
                    raise ParsingError(
                        "The first directive must be 'nb_drones'.",
                        line_num
                    )
                first_non_empty_seen = True
            if line.startswith("nb_drones:"):
                self._parse_nb_drones(line, line_num)
            elif any(
                line.startswith(prefix)
                for prefix in ("hub", "start_hub", "end_hub")
            ):
                self._parse_hub(line, line_num)
            elif line.startswith("connection:"):
                self._parse_connection(line, line_num)
            else:
                raise ParsingError(
                    f"Unknown directive or malformed line: '{line}'",
                    line_num
                )
        self._validate_map_data()
        return self.map_data

    def _validate_map_data(self) -> None:
        if self.map_data.nb_drones <= 0:
            raise ParsingError("The 'nb_drones' directive must be a positive integer.")
        if self.map_data.start_hub is None:
            raise ParsingError("No 'start_hub' defined in the map.")
        if self.map_data.end_hub is None:
            raise ParsingError("No 'end_hub' defined in the map.")
        if not self.map_data.connections:
            raise ParsingError("No connections defined in the map.")

    def _parse_connection(self, line: str, line_num: int) -> None:
        rest = line.split(":", 1)[1].strip()
        clean_rest, raw_attributes = self._extract_attributes(rest, line_num)
        parts = clean_rest.split("-")
        if len(parts) != 2:
            raise ParsingError(
                f"Malformed connection definition: expected 'source-target', got '{clean_rest}'.",
                line_num
            )
        source, target = parts[0].strip(), parts[1].strip()
        if not source or not target:
            raise ParsingError(
                f"Malformed connection definition: source and target cannot be empty.",
                line_num
            )
        if source not in self.map_data.hubs:
            raise ParsingError(
                f"Connection source hub '{source}' is not defined.",
                line_num
            )
        if target not in self.map_data.hubs:
            raise ParsingError(
                f"Connection target hub '{target}' is not defined.",
                line_num
            )
        if source == target:
            raise ParsingError(
                f"Connection cannot be self-referential: '{source}' to '{target}'.",
                line_num
            )
        edge_key = tuple(sorted([source, target]))
        if edge_key in self.seen_connections:
            raise ParsingError(
                f"Duplicate connection between '{source}' and '{target}' found.",
                line_num
            )
        self.seen_connections.add(edge_key)
        max_link_capacity = 1
        for key, value in raw_attributes.items():
            if key == "max_link_capacity":
                try:
                    max_link_capacity = int(value)
                    if max_link_capacity <= 0:
                        raise ValueError
                except ValueError:
                    raise ParsingError(
                        f"Invalid 'max_link_capacity' value '{value}' for connection "
                        f"between '{source}' and '{target}': must be a positive integer.",
                        line_num
                    )
            else:
                raise ParsingError(
                    f"Unknown attribute '{key}' for connection between '{source}' and '{target}'.",
                    line_num
                )
        connection = Connection(
            source=source,
            target=target,
            max_link_capacity=max_link_capacity
        )
        self.map_data.connections.append(connection)

    def _parse_hub(self, line: str, line_num: int) -> None:
        hub_type, rest = line.split(":", 1)
        hub_type = hub_type.strip()
        rest = rest.strip()
        clean_rest, raw_attributes = self._extract_attributes(rest, line_num)
        tokens = clean_rest.split()
        if len(tokens) != 3:
            raise ParsingError(
                f"Malformed hub definition: expected 3 tokens, got {len(tokens)}.",
                line_num
            )
        name, x_str, y_str = tokens
        if "-" in name:
            raise ParsingError(
                f"Invalid hub name '{name}': names cannot contain dashes.",
                line_num
            )
        if name in self.map_data.hubs:
            raise ParsingError(
                f"Duplicate hub name '{name}' found.",
                line_num
            )
        try:
            x = int(x_str)
            y = int(y_str)
        except ValueError:
            raise ParsingError(
                f"Invalid coordinates for hub '{name}': x and y must be integers.",
                line_num
            )
        zone = "normal"
        color: str | None = None
        max_drones = 1
        for key, value in raw_attributes.items():
            if key == "zone":
                zone_value = value.lower()
                if zone_value not in ALLOWED_ZONE_TYPES:
                    raise ParsingError(
                        f"Invalid zone type '{value}' for hub '{name}'. "
                        f"Allowed types: {', '.join(ALLOWED_ZONE_TYPES)}.",
                        line_num
                    )
                zone = zone_value
            elif key == "color":
                color = value
            elif key == "max_drones":
                try:
                    max_drones = int(value)
                    if max_drones <= 0:
                        raise ValueError
                except ValueError:
                    raise ParsingError(
                        f"Invalid 'max_drones' value '{value}' for hub '{name}': "
                        "must be a positive integer.",
                        line_num
                    )
            else:
                raise ParsingError(
                    f"Unknown attribute '{key}' for hub '{name}'.",
                    line_num
                )
        hub = Hub(
            name=name,
            hub_type=hub_type,
            x=x,
            y=y,
            zone=zone,
            color=color,
            max_drones=max_drones
        )
        self.map_data.hubs[name] = hub
        if hub_type == "start_hub":
            if self.map_data.start_hub is not None:
                raise ParsingError(
                    "Multiple 'start_hub' definitions found.",
                    line_num
                )
            self.map_data.start_hub = hub
        elif hub_type == "end_hub":
            if self.map_data.end_hub is not None:
                raise ParsingError(
                    "Multiple 'end_hub' definitions found.",
                    line_num
                )
            self.map_data.end_hub = hub

    def _extract_attributes(self, text: str, line_num: int) -> tuple[str, dict]:
        clean_text = text
        attributes = {}
        if "[" in text or "]" in text:
            match = re.search(r"\[(.*?)\]", text)
            if not match or text.count("[") != 1 or text.count("]") != 1:
                raise ParsingError(
                    "Malformed attributes: must contain exactly one pair of brackets.",
                    line_num
                )
            attr_text = match.group(1).strip()
            clean_text = (text[:match.start()] + text[match.end():]).strip()
            if attr_text:
                for token in attr_text.split():
                    if "=" not in token:
                        raise ParsingError(
                            f"Malformed attribute '{token}': must be in key=value format.",
                            line_num
                        )
                    key, value = token.split("=", 1)
                    key = key.strip()
                    value = value.strip()
                    if not key or not value:
                        raise ParsingError(
                            f"Malformed attribute '{token}': key and value cannot be empty.",
                            line_num
                        )
                    attributes[key] = value
        return clean_text, attributes

    def _parse_nb_drones(self, line: str, line_num: int) -> None:
        if self.map_data.nb_drones != 0:
            raise ParsingError(
                "Duplicate 'nb_drones' directive found.",
                line_num
            )
        parts = line.split(":", 1)
        value_str = parts[1].strip()
        try:
            value = int(value_str)
            if value <= 0:
                raise ValueError
        except ValueError:
            raise ParsingError(
                f"Invalid 'nb_drones' value: '{value_str}'. Must be a positive integer.",
                line_num
            )
        self.map_data.nb_drones = value

