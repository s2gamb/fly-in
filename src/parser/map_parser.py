"""Parser for Fly-in drone simulation map files."""

import re
from typing import Dict, Set, Tuple, Optional

from .models import MapData, Hub, Connection, ParsingError

ALLOWED_ZONE_TYPES: Set[str] = {"normal", "blocked", "restricted", "priority"}


class MapParser:
    """Parses and validates map configuration files according to Fly-in rules."""

    def __init__(self) -> None:
        """Initialize the MapParser with clean state."""
        self.map_data: MapData = MapData()
        self.seen_connections: Set[Tuple[str, str]] = set()

    def parse_file(self, file_path: str) -> MapData:
        """Parse a map file from a path and return validated MapData.

        Args:
            file_path: Path to the map file.

        Returns:
            Validated MapData instance.

        Raises:
            ParsingError: If any syntax or semantic constraint is violated.
        """
        with open(file_path, "r", encoding="utf-8") as f:
            return self.parse_text(f.read())

    def parse_text(self, content: str) -> MapData:
        """Parse map configuration from a text string.

        Args:
            content: Raw map file content.

        Returns:
            Validated MapData instance.

        Raises:
            ParsingError: If any syntax or semantic constraint is violated.
        """
        lines = content.splitlines()
        first_non_empty_seen = False

        for line_num, raw_line in enumerate(lines, 1):
            # Strip comments and surrounding whitespace
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue

            if not first_non_empty_seen:
                if not line.startswith("nb_drones:"):
                    raise ParsingError(
                        "The first directive must define the number of drones using 'nb_drones: <positive_integer>'",
                        line_num=line_num,
                    )
                first_non_empty_seen = True

            if line.startswith("nb_drones:"):
                self._parse_nb_drones(line, line_num)
            elif any(
                line.startswith(prefix)
                for prefix in ("start_hub:", "hub:", "end_hub:")
            ):
                self._parse_hub(line, line_num)
            elif line.startswith("connection:"):
                self._parse_connection(line, line_num)
            else:
                raise ParsingError(
                    f"Unrecognized syntax or directive: '{line}'",
                    line_num=line_num,
                )

        self._validate_final_state()
        return self.map_data

    def _parse_nb_drones(self, line: str, line_num: int) -> None:
        """Parse and validate nb_drones directive."""
        if self.map_data.nb_drones > 0:
            raise ParsingError("Duplicate 'nb_drones' directive", line_num)

        parts = line.split(":", 1)
        val_str = parts[1].strip()
        try:
            val = int(val_str)
            if val <= 0:
                raise ValueError
        except ValueError:
            raise ParsingError(
                f"Invalid drone count '{val_str}': must be a positive integer",
                line_num=line_num,
            )
        self.map_data.nb_drones = val

    def _extract_metadata(self, text: str, line_num: int) -> Tuple[str, Dict[str, str]]:
        """Extract metadata in brackets '[...]' and return clean text and key-value dict."""
        clean_text = text
        attributes: Dict[str, str] = {}

        if "[" in text or "]" in text:
            match = re.search(r"\[(.*?)\]", text)
            if not match or text.count("[") != 1 or text.count("]") != 1:
                raise ParsingError("Malformed metadata bracket syntax", line_num)

            meta_str = match.group(1).strip()
            clean_text = (text[:match.start()] + text[match.end():]).strip()

            if meta_str:
                for token in meta_str.split():
                    if "=" not in token:
                        raise ParsingError(
                            f"Invalid metadata entry '{token}', expected 'key=value'",
                            line_num,
                        )
                    key, val = token.split("=", 1)
                    key = key.strip()
                    val = val.strip()
                    if not key or not val:
                        raise ParsingError(
                            f"Empty key or value in metadata '{token}'",
                            line_num,
                        )
                    attributes[key] = val

        return clean_text, attributes

    def _parse_hub(self, line: str, line_num: int) -> None:
        """Parse and validate a hub definition."""
        hub_type, rest = line.split(":", 1)
        hub_type = hub_type.strip()
        rest = rest.strip()

        clean_rest, raw_attrs = self._extract_metadata(rest, line_num)
        tokens = clean_rest.split()

        if len(tokens) != 3:
            raise ParsingError(
                f"Invalid hub declaration '{line}'. Expected '<name> <x> <y>'",
                line_num,
            )

        name, x_str, y_str = tokens

        if "-" in name:
            raise ParsingError(
                f"Hub name '{name}' contains forbidden character '-'",
                line_num,
            )

        if name in self.map_data.hubs:
            raise ParsingError(f"Duplicate hub name '{name}'", line_num)

        try:
            x = int(x_str)
            y = int(y_str)
        except ValueError:
            raise ParsingError(
                f"Hub '{name}' coordinates must be valid integers (got '{x_str}', '{y_str}')",
                line_num,
            )

        # Validate hub metadata
        zone = "normal"
        color: Optional[str] = None
        max_drones = 1

        for key, val in raw_attrs.items():
            if key == "zone":
                zone_val = val.lower()
                if zone_val not in ALLOWED_ZONE_TYPES:
                    raise ParsingError(
                        f"Invalid zone type '{val}'. Must be one of: {sorted(ALLOWED_ZONE_TYPES)}",
                        line_num,
                    )
                zone = zone_val
            elif key == "color":
                if len(val.split()) != 1:
                    raise ParsingError(
                        f"Color '{val}' must be a single-word string",
                        line_num,
                    )
                color = val
            elif key == "max_drones":
                try:
                    cap = int(val)
                    if cap <= 0:
                        raise ValueError
                    max_drones = cap
                except ValueError:
                    raise ParsingError(
                        f"max_drones '{val}' must be a positive integer",
                        line_num,
                    )
            else:
                raise ParsingError(
                    f"Unknown hub metadata key '{key}'",
                    line_num,
                )

        hub = Hub(
            name=name,
            hub_type=hub_type,
            x=x,
            y=y,
            zone=zone,
            color=color,
            max_drones=max_drones,
            attributes=raw_attrs,
        )

        self.map_data.hubs[name] = hub

        if hub_type == "start_hub":
            if self.map_data.start_hub is not None:
                raise ParsingError("Map cannot contain multiple 'start_hub' zones", line_num)
            self.map_data.start_hub = hub
        elif hub_type == "end_hub":
            if self.map_data.end_hub is not None:
                raise ParsingError("Map cannot contain multiple 'end_hub' zones", line_num)
            self.map_data.end_hub = hub

    def _parse_connection(self, line: str, line_num: int) -> None:
        """Parse and validate a connection definition."""
        rest = line.split(":", 1)[1].strip()
        clean_rest, raw_attrs = self._extract_metadata(rest, line_num)

        parts = clean_rest.split("-")
        if len(parts) != 2:
            raise ParsingError(
                f"Invalid connection format '{clean_rest}'. Expected '<zone1>-<zone2>'",
                line_num,
            )

        u = parts[0].strip()
        v = parts[1].strip()

        if not u or not v:
            raise ParsingError(f"Invalid connection hubs in '{clean_rest}'", line_num)

        if u not in self.map_data.hubs:
            raise ParsingError(
                f"Connection references undefined hub '{u}'",
                line_num,
            )
        if v not in self.map_data.hubs:
            raise ParsingError(
                f"Connection references undefined hub '{v}'",
                line_num,
            )

        if u == v:
            raise ParsingError(
                f"Self-connection on hub '{u}' is not allowed",
                line_num,
            )

        edge_key = tuple(sorted([u, v]))
        if edge_key in self.seen_connections:
            raise ParsingError(
                f"Duplicate connection between '{u}' and '{v}'",
                line_num,
            )
        self.seen_connections.add(edge_key)

        max_link_capacity = 1
        for key, val in raw_attrs.items():
            if key == "max_link_capacity":
                try:
                    cap = int(val)
                    if cap <= 0:
                        raise ValueError
                    max_link_capacity = cap
                except ValueError:
                    raise ParsingError(
                        f"max_link_capacity '{val}' must be a positive integer",
                        line_num,
                    )
            else:
                raise ParsingError(
                    f"Unknown connection metadata key '{key}'",
                    line_num,
                )

        conn = Connection(
            source=u,
            target=v,
            max_link_capacity=max_link_capacity,
            attributes=raw_attrs,
        )
        self.map_data.connections.append(conn)

    def _validate_final_state(self) -> None:
        """Ensure mandatory elements (start_hub, end_hub) are present and valid."""
        if self.map_data.nb_drones <= 0:
            raise ParsingError("Map does not define any drones ('nb_drones')")
        if self.map_data.start_hub is None:
            raise ParsingError("Map is missing a 'start_hub' definition")
        if self.map_data.end_hub is None:
            raise ParsingError("Map is missing an 'end_hub' definition")
        if self.map_data.start_hub.name == self.map_data.end_hub.name:
            raise ParsingError("start_hub and end_hub cannot be the same zone")


def parse_map_file(file_path: str) -> MapData:
    """Convenience helper to parse a map file using MapParser.

    Args:
        file_path: Path to the map file.

    Returns:
        Validated MapData instance.
    """
    parser = MapParser()
    return parser.parse_file(file_path)

