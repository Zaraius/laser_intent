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

For direct, uncalibrated mirror positioning, send normalized X/Y coordinates:

```bash
ros2 topic pub --once /laser_command std_msgs/msg/String "{data: 'raw 0.25 -0.5'}"
```

`raw` bypasses surface projection and calibration and draws a stationary spot
at the requested position. Its color follows the `color_r`, `color_g`, and
`color_b` parameters and `/laser_color` topic, just like the arrow demo. Both
coordinates must be in `[-1, 1]`. This is a raw DAC-field position, not a
physical floor or wall coordinate.

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
projector height or attitude is hard-coded. The current core calibration is a
scale/offset mapping. `Calibration2D` is a deliberately unimplemented
surface-to-mirror calibration scaffold; its methods fail explicitly and it is
not yet connected to the ROS node.

For bench testing without surface projection or calibration, the core also
provides `Projector::show_raw_frame`. Its X/Y inputs are normalized mirror
coordinates in `[-1, 1]`, not physical distances or integer DAC counts. The
Helios backend maps these endpoints to its 16-bit coordinate range using the
SDK's `WriteFrameHighResolution` API; older libraries that only export
`WriteFrame` are not supported. This avoids coordinate wraparound when the
legacy API is used with network DACs. UDP forwards the normalized values. Raw
frames must begin and end with blanked points; callers are responsible for
blanked travel and safe, in-range paths.
The ROS `raw x y` command sends a repeated point with blanked endpoints to
render a colored spot.

The Helios backend now refuses frames until initialization succeeds, bounds DAC
readiness waits to one second, and blacks out on invalid or empty paths. The
node logs backend initialization failure; it does not claim hardware is ready.

## Limitations

V1 renders flat-plane paths and optional directional arrows. The Helios backend
expects a `libHeliosDacAPI.so` that exports `WriteFrameHighResolution` at
runtime. Text, mesh surfaces, distortion calibration, and lifecycle management
are intentionally deferred until the demo proves the data path.

start tf2:
ros2 run tf2_ros static_transform_publisher   0 0 1 0 0 0 map laser_projector

start laser:
ros2 run laser_projection_ros laser_projection_node --ros-args   -p backend:=helios   -p helios_library:=/home/agx/helios_dac/sdk/cpp/shared_library/libHeliosDacAPI.so.new -p scan_rate:=15000.0 -p point_spacing:=0.001

Draw an arrow:

ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: map}, poses: [{header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.995, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.95, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.1, y: -0.952, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.95, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.04, y: -0.952, z: 0.0}, orientation: {w: 1.0}}}]}"

ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: map}, poses: [{header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.98, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.96, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.085, y: -0.962, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.96, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.055, y: -0.962, z: 0.0}, orientation: {w: 1.0}}}]}"

draw a single point
ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: map}, poses: [{header: {frame_id: map}, pose: {position: {x: 0.07, y: -0.95, z: 0.00}, orientation: {w: 1.0}}}]}"

 ros2 run laser_projection_ros laser_projection_node --ros-args -p backend:=helios -p helios_library:=/home/agx/helios_dac/sdk/cpp/shared_library/libHeliosDacAPI.so.new -p scan_rate:=15000.0 -p point_spacing:=0.001 -p color_r:=0 -p color_g:=100 -p color_b:=0

 ## SVG to Helios

 The ROS node accepts `svg <path>` on `/laser_command` and sends the rendered
 raw-coordinate frame through its configured backend. For Helios, start the
 same node used for the arrow demo:

 ```bash
 source /opt/ros/jazzy/setup.bash
 cd /home/agx/ros2_jazzy
 colcon build --packages-up-to laser_projection_ros --symlink-install
 source install/setup.bash
 ros2 run laser_projection_ros laser_projection_node --ros-args \
   -p backend:=helios \
   -p helios_library:=/home/agx/helios_dac/sdk/cpp/shared_library/libHeliosDacAPI.so.new \
   -p scan_rate:=15000.0 \
   -p svg_bottom_left_x:=-0.8 -p svg_bottom_left_y:=-0.8 \
   -p svg_top_right_x:=0.8 -p svg_top_right_y:=0.8 \
   -p svg_scale_x:=1.0 -p svg_scale_y:=1.0 \
   -p svg_rotation_degrees:=0.0
 ```

 In another terminal, send an SVG. This draws the bundled face sample:

 ```bash
 source /opt/ros/jazzy/setup.bash
 ros2 topic pub --once /laser_command std_msgs/msg/String \
   "{data: 'svg /home/agx/ros2_jazzy/src/laser_intent/test_images/face.svg'}"
 ```

 SVG rendering uses raw mirror coordinates and does not require the projector
 TF. It uses the node's already-initialized Helios backend directly; `svg`
 does not go through the standalone UDP sender. The requested box must be
 ordered and within `[-1, 1]`. Artwork preserves aspect ratio, is centered in
 the box, and SVG top/bottom map to the box's top/bottom. The X/Y scale factors
 are applied around the box center. If scaling would exceed `[-1, 1]`, the
 whole image is uniformly reduced until it fits.

 Parameters `svg_bottom_left_x/y`, `svg_top_right_x/y`, `svg_scale_x/y`,
 `svg_rotation_degrees`, `svg_hatch_spacing`, `svg_point_spacing`, and
 `svg_max_points` control placement and rendering (`svg_max_points` defaults to
 `500000`). Positive rotation is counterclockwise, around the image center.
 Parameters are read for each `svg` command, so they can be adjusted while the
 node is running, then the SVG command can be sent again. For example:

 ```bash
 ros2 param set /laser_projection_node svg_scale_x 0.7
 ros2 param set /laser_projection_node svg_scale_y 0.7
 ros2 param set /laser_projection_node svg_rotation_degrees 15.0
 ros2 topic pub --once /laser_command std_msgs/msg/String \
   "{data: 'svg /home/agx/ros2_jazzy/src/laser_intent/test_images/face.svg'}"
 ```

 For an image
 with empty or spaced paths, its colors come from the SVG; opacity is ignored.
 `clear` turns the beam off. Observe the laser enclosure, interlocks, and beam
 stop precautions before sending any drawing.

 The renderer supports SVG paths (`M/L/H/V/C/S/Q/T/A/Z`), lines, polylines,
 polygons, rectangles, circles, ellipses, nested groups, common transforms,
 solid hex/RGB colors and common color names, and fill/stroke styling. Filled
 shapes are rendered as horizontal scan-line hatches; tune their spacing in raw
 units with `svg_hatch_spacing` (default `0.1`). `svg_point_spacing` controls
 the maximum distance between output points (default `0.05`).
 Stroke width is treated as a centerline, SVG opacity is ignored with a warning,
 and unsupported SVG elements are reported and skipped. Gradients, text,
 clipping, and embedded images are not rendered; clipping and masks produce
 warnings and are not applied. The core library also provides a standalone UDP
 sender (`ros2 run laser_projection_core laser_svg_sender --help`) for use with
 the existing `LASER_FRAME` receiver; use the ROS command above to send to
 Helios through the laser node. Invalid settings, oversized frames, and
 malformed SVGs are reported instead of silently clipped.