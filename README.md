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

## Hardware arrow demo

Rebuild after code changes:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zaraius/ros2_ws/src/laser_intent
colcon build --packages-select laser_projection_ros --symlink-install
```

Start the node (assuming the projector TF is already being published):

```bash
ros2 run laser_projection_ros laser_projection_node --ros-args \
  -p backend:=helios \
  -p helios_library:=/home/agx/helios_dac/sdk/cpp/shared_library/libHeliosDacAPI.so.new \
  -p scan_rate:=2000.0 -p point_spacing:=0.001
```

In another terminal, draw one arrow and turn it off:

```bash
ros2 topic pub --once /laser_command std_msgs/msg/String "{data: 'arrow'}"
ros2 topic pub --once /laser_command std_msgs/msg/String "{data: 'clear'}"
```

The demo arrow uses the measured points: tail `(0.07, -0.98)`, junction
`(0.07, -0.96)`, and head corners `(0.085, -0.962)` and
`(0.055, -0.962)`, on the `surface_z` plane. Use `arrow 2` through `arrow 20`
to fit that many narrower arrows across the same measured X range. The frame
adds blanked travel back to its start so the repeating scan does not draw a
stray closing line. Green is intentional; the default is
`color_r:=0 color_g:=255 color_b:=0`. `clear` sends a black fixed-point frame.

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

start tf2:
ros2 run tf2_ros static_transform_publisher   0 0 1 0 0 0 map laser_projector

start laser:
ros2 run laser_projection_ros laser_projection_node --ros-args   -p backend:=helios   -p helios_library:=/home/agx/helios_dac/sdk/cpp/shared_library/libHeliosDacAPI.so.new -p scan_rate:=15000.0 -p point_spacing:=0.001

Draw an arrow:

ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: map}, poses: [{header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.995, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.95, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.1, y: -0.952, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.95, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.04, y: -0.952, z: 0.0}, orientation: {w: 1.0}}}]}"

ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: map}, poses: [{header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.98, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.96, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.085, y: -0.962, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.96, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.055, y: -0.962, z: 0.0}, orientation: {w: 1.0}}}]}"

draw a single point
ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: map}, poses: [{header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.95, z: 0.00}, orientation: {w: 1.0}}}]}"
