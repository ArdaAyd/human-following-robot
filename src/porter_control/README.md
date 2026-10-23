# Porter control nodes

## Colour detector

`color_detector` converts the RGB camera image to HSV, cleans the orange mask,
and selects its largest external contour. The target is accepted only when its
area meets `min_area`. A high saturation threshold rejects the brown warehouse
crates while retaining the shaded orange vest.

| Topic | Meaning |
|---|---|
| `/camera/image_raw` | Input image; reliable subscription, queue depth 1 |
| `/target` | `PointStamped`: X = bearing (rad, left positive), Y = contour area (px), Z unused |
| `/target_debug_image` | RGB mask, image centre line and accepted target centroid |

The `/target` payload is a bearing/area protocol, **not a Cartesian point**.
Its header retains the source image timestamp and uses `camera_link`, whose
axes are +X forward and +Y left. The debug image retains the input image's
header, including `camera_optical_link`. No `/target` message is emitted when
the target is absent or too small; the debug image continues with `NO TARGET`.
Malformed frames are skipped with a throttled warning. Target loss and motion
commands belong to `follow_controller`.

The bearing is `atan((W/2 - u) / f)`, with
`f = (W/2) / tan(HFOV/2)`. `hfov_deg` must match `camera.xacro` (80 degrees).
All settings in `config/color_detector.yaml` are validated, read-only startup
parameters. Restart the node after editing them. OpenCV hue is 0-179; saturation
and value are 0-255. This is simple colour detection: another sufficiently large
orange object can become the target; there is no identity recognition.

### Run and inspect

Build and source the workspace, then start the world, robot, shared bridge and
camera viewer using the [camera guide](../porter_description/README.md).
In another sourced terminal, run:

```bash
ros2 run porter_control color_detector --ros-args \
  --params-file "$(ros2 pkg prefix porter_control)/share/porter_control/config/color_detector.yaml"
```

Open the live mask in another sourced terminal:

```bash
ros2 run rqt_image_view rqt_image_view /target_debug_image
```

Inspect the bearing with `ros2 topic echo /target`. A target to the robot's left
must give a positive X value; a target to its right must give a negative value.
The green cross marks the accepted centroid. When the vest leaves the image,
`/target` must fall silent and the debug view must show `NO TARGET`; returning
the vest must resume detections. Keep `person_walker` stopped during this check.
The detector does not move the robot.

### Detector tests

```bash
python3 -m pytest src/porter_control/test/test_color_detector*.py
```

The tests cover resolution-independent bearing, largest-contour selection,
brown/blue rejection, shaded orange, area thresholds, invalid settings, RGB/BGR
conversion, original timestamps, target loss, debug output and recovery after a
malformed image. They also check clean shutdown without a camera stream and
with repeated interrupt signals. The ROS tests use private topic remappings.

## Person walker

`person_walker` publishes `geometry_msgs/msg/Twist` on `/person/cmd_vel` at
20 Hz. The supplied configuration uses Gazebo's ROS simulation clock. The
command heartbeat continues during a pause, but elapsed scenario time freezes.
The sequence waits for a nonzero simulation clock and a command subscriber
(normally the ROS-to-Gazebo bridge), then starts its delay. Rewinding the clock
restarts the sequence.

### Default scenario

Times are relative to readiness, not to world startup. All motion is expressed
in the person's body frame: +X forward, +Y left, +Z yaw counterclockwise.

| Time (s) | Phase | Command |
|---|---|---|
| 0-2 | Start delay | Stop |
| 2-14 | Straight | 0.3 m/s forward (3.6 m) |
| 14-17 | First turn | 0.5236 rad/s left (90 degrees) |
| 17-27 | Approach | 0.3 m/s forward (3 m) |
| 27-30 | Second turn | 0.5236 rad/s left (90 degrees) |
| 30-52 | Cross yard | 0.3 m/s forward (6.6 m) |
| 52-55 | Third turn | 0.5236 rad/s left (90 degrees) |
| 55-69 | Ramp leg | 0.3 m/s forward (4.2 m) |
| 69-72 | Sudden sidestep | 0.3 m/s left (0.9 m) |
| 72 onward | Stopped | Repeated zero velocity |

The default route covers 18.3 m without increasing the walking speed. In
`porter_world.sdf` it follows (0, 0), (3.6, 0), (3.6, 3), (-3, 3),
(-3, -1.2), then (-2.1, -1.2). The roughly 12 x 11 m yard has obstacles
clear of the route and a south-facing ramp on the last forward leg. The person
finishes on the 0.25 m high deck. The ramp leg is a timed command, not terrain
detection; changing speeds or durations also requires realigning the world.
Camera-view loss during the sidestep needs checking after camera/controller
integration. The sequence runs once; reset the person pose and restart the node
to repeat from the same starting point.

### Run with the person model

Build from the workspace root and load the environment:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Start the project world, which already includes the person. The models resource
path lets Gazebo resolve the world's relative person include; the world name
matches the clock entry in the shared bridge:

```bash
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix porter_gazebo)/share/porter_gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r "$(ros2 pkg prefix porter_gazebo)/share/porter_gazebo/worlds/porter_world.sdf"
```

In a separate terminal, source ROS and the workspace as above, then start the
single shared bridge configuration:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$(ros2 pkg prefix porter_gazebo)/share/porter_gazebo/config/bridge.yaml"
```

In another sourced terminal, start the scenario:

```bash
ros2 run porter_control person_walker --ros-args \
  --params-file "$(ros2 pkg prefix porter_control)/share/porter_control/config/person_walker.yaml"
```

Inspect the output with `ros2 topic echo /person/cmd_vel` or
`ros2 topic hz /person/cmd_vel`. The model should traverse the yard, make three
left turns, climb the ramp, sidestep, and stop. Keep the node running to continue publishing the final stop.
Ctrl+C or SIGTERM sends zero velocity and waits up to 0.5 s for acknowledgements
before ROS shuts down.
A forced kill cannot send a stop; recover with:

```bash
gz topic -t /person/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0} angular: {z: 0}'
```

Speeds, durations, start delay, and publish rate are read-only startup parameters
in `config/person_walker.yaml`; edit them and restart the node to tune the demo.
Durations and speeds must be finite and non-negative, and publish rate positive.
Set `use_sim_time:=false` explicitly only for command-only tests without Gazebo.

### Tests

From a sourced workspace, run the walker tests with:

```bash
python3 -m pytest src/porter_control/test/test_person_walker*.py
```

These cover phase boundaries, skipped phases, invalid settings, receiver
readiness, paused simulation time, clock rewind, and final stop delivery.
The ROS test remaps its clock and command topics to a private test prefix.
