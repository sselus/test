# Architecture

## Nodes and Topics

- `patrol_node`
- Publishes `patrol/status` (`std_msgs/String`)
- Subscribes `patrol/stop` (`std_msgs/Bool`)

## Core Modules

- `waypoint_loader.py` handles YAML waypoint parsing.
- `safety_monitor.py` debounces emergency stop state.
