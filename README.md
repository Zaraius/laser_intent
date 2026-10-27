# laser_intent

Small ROS 2 Jazzy demo for projecting a navigation path into laser points.

```text
nav_msgs/Path -> TF2 -> laser_projection_core -> UDP or Helios
```

`laser_projection_core` has no ROS dependency. `laser_projection_ros` only
handles ROS messages, TF2, parameters, and debug markers.

The existing `osc2laser` project was inspected but not copied. Its useful
parts are `osc-receiver/models.py` (point/object concepts) and
`osc-receiver/laser_output.py` (the Helios point layout and calls to
`OpenDevices`, `GetStatus`, and `WriteFrame`). It is AGPL-3.0; this demo
instead uses a clean-room, minimal Helios ABI declaration and does not link
against or copy its Python receiver. The OSC/UI files, preview, optimizer,
and sender projects are not core dependencies.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd laser_intent
colcon build --symlink-install
source install/setup.bash
```

## Run the ROS node

```bash
ros2 run laser_projection_ros laser_projection_node \
  --ros-args -p backend:=udp -p path_topic:=/plan \
  -p projection_frame:=map -p projector_frame:=laser_projector
```

For a hardware center-dot test, first publish the projector TF and start the
node with `-p backend:=helios -p color_r:=255 -p color_g:=255 -p color_b:=255`.
Then publish a `nav_msgs/msg/Path` containing one point at the floor origin.
The point remains at normalized laser coordinates `(0, 0)`.

The path topic is runtime state: publish another path to move or replace the
shape. Publish `std_msgs/msg/ColorRGBA` on `/laser_color` to change RGB
dynamically; values are normalized from 0.0 to 1.0 and the last path is
resent automatically:

```bash
ros2 topic pub --once /laser_color std_msgs/msg/ColorRGBA \
"{r: 1.0, g: 1.0, b: 1.0, a: 1.0}"
```

An empty path blacks out the backend. Preloaded shapes are not included yet;
paths are the one runtime shape primitive in this demo.

The node publishes `visualization_msgs/Marker` on
`~/debug_path`. The UDP backend sends one ASCII frame to `127.0.0.1:2345`:

```text
LASER_FRAME <scan_rate> <count>\n
<x> <y> <r> <g> <b> <blank>\n
...
```

This is intentionally separate from osc2laser's OSC protocol. The existing
OSC receiver accepts object/effect messages, not arbitrary point frames.

## Core API

```cpp
auto backend = std::make_shared<laser_projection::UdpBackend>("127.0.0.1", 2345);
laser_projection::Projector projector(
    std::make_shared<laser_projection::PlaneSurface>(0.0), backend);
projector.show_path(path_points, {{0.0, 0.0, 1.0}, {}});
```

Projector TF supplies the actual mounting position and orientation; no
projector height or attitude is hard-coded. Calibration is a small
scale/offset mapping and can be replaced later without changing the ROS node.

The Helios backend now refuses frames until initialization succeeds, bounds DAC
readiness waits to one second, and blacks out on invalid or empty paths. The
node logs backend initialization failure; it does not claim hardware is ready.

## Limitations

V1 renders flat-plane paths and optional directional arrows. The Helios backend
expects a compatible `libHeliosDacAPI.so` at runtime. Text, mesh surfaces,
distortion calibration, and lifecycle management are intentionally deferred
until the demo proves the data path.
