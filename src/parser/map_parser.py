import re

from .models import MapData, Hub, Connection


def parse_attributes(attr_str: str) -> dict[str, str]:
    attributes = {}
    if not attr_str:
        return attributes
    
    matches = re.findall(r"(\w+)=([^\s\]]+)", attr_str)
    for key, value in matches:
        attributes[key] = value
    return attributes


def parse_map_file(file_path: str) -> MapData:
    map_data = MapData()

    with open(file_path, "r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.split("#")[0].strip()
            if not line:
                continue

            if line.startswith("nb_drones:"):
                val = line.split(":", 1)[1].strip()
                map_data.nb_drones = int(val)
            
            elif any(
                line.startswith(prefix)
                for prefix in ("start_hub:", "hub:", "end_hub:")
            ):
                hub_type, rest = line.split(":", 1)
                hub_type = hub_type.strip()

                attr_match = re.search(r"\[(.*?)\]", rest)
                attr_str = attr_match.group(1) if attr_match else ""

                clean_rest = re.sub(r"\[.*?\]", "", rest).strip()
                parts = clean_rest.split()

                if len(parts) >= 3:
                    name = parts[0]
                    x = float(parts[1])
                    y = float(parts[2])
                    attributes = parse_attributes(attr_str)
                
                    hub = Hub(
                        name=name,
                        hub_type=hub_type,
                        x=x,
                        y=y,
                        attributes=attributes
                    )
                    map_data.hubs[name] = hub

                    if hub_type == "start_hub":
                        map_data.start_hub = hub
                    elif hub_type == "end_hub":
                        map_data.end_hub = hub
            
            elif line.startswith("connection:"):
                rest = line.split(":", 1)[1].strip()

                attr_match = re.search(r"\[(.*?)\]", rest)
                attr_str = attr_match.group(1) if attr_match else ""

                clean_rest = re.sub(r"\[.*?\]", "", rest).strip()
                nodes = clean_rest.split("-")

                if len(nodes) == 2:
                    conn = Connection(
                        source=nodes[0].strip(),
                        target=nodes[1].strip(),
                        attributes=parse_attributes(attr_str),
                    )
                    map_data.connections.append(conn)

    return map_data
