# One-command Porter bringup

Stop any earlier Porter launch or manual Gazebo/bridge session before starting.
Run one session per ROS domain and Gazebo partition so `/clock`, TF and sensor
messages come from the same simulation.

From the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch porter_gazebo bringup.launch.py
```

The launch starts the packaged `porter_world` with its existing person, publishes
Porter's Xacro description and spawns one robot at `(-1, 0, 0.01)` on clear ground.
The models resource path is set automatically, preserving any existing entries.
Once spawning succeeds, it starts the single shared bridge, the prepared RViz
view, colour detection and the 72-second person scenario. Startup uses process
and spawn events rather than fixed delays. A failed spawn stops bringup before
the bridge and demo nodes start.

Gazebo's server and GUI are separate child processes of this launch. Closing the
Gazebo window or pressing Ctrl+C in the launch terminal stops the session and its
child processes. Closing RViz alone leaves the simulation running.

The default session runs the complete demonstration: the person walks the
72-second route and Porter follows at the configured distance, over the ramp and
onto the loading deck. Pass `follow:=false` to walk the person while Porter stays
in place, which is the easier view when inspecting the camera and detector mask.

## Launch options

| Argument | Default | Purpose |
|---|---|---|
| `gui` | `true` | Open the Gazebo window; false uses headless rendering |
| `rviz` | `true` | Open `porter_description/rviz/porter.rviz` |
| `behaviour` | `true` | Enable detection and the selected demo/controller nodes |
| `walker` | `true` | Run `person_walker` when behaviour is enabled |
| `follow` | `true` | Run the following controller when behaviour is enabled |

Keep the person stationary while inspecting the camera and mask:

```bash
ros2 launch porter_gazebo bringup.launch.py walker:=false
```

Open only the world, robot and sensor visualisation:

```bash
ros2 launch porter_gazebo bringup.launch.py behaviour:=false
```

Run sensor publishers without either GUI or behaviour nodes:

```bash
ros2 launch porter_gazebo bringup.launch.py gui:=false rviz:=false behaviour:=false
```

`behaviour:=false` disables all three optional nodes, regardless of the `walker`
and `follow` arguments. All ROS nodes launched here use simulation time. The
world name is fixed to match the clock mapping in the shared bridge.

## RViz view

The prepared view uses `odom` as its fixed frame and follows `base_footprint`.
It contains:

- **Porter:** the robot model from the transient-local `/robot_description` topic.
- **LiDAR:** cyan `/scan` points in the 3D view, transformed from `lidar_link`.
- **Camera image:** the live 640 x 480 RGB image. Camera axes and field-of-view
  edges show the mounting position and calibration in 3D.
- **IMU orientation and acceleration:** axes and an orange acceleration arrow
  from `/imu`, provided by `rviz_imu_plugin`. Its arrow scale is 0.05 m per
  m/s²; the simulated accelerometer reads about +9.8 m/s² on Z at rest.
- **Vest mask:** an optional image display for `/target_debug_image`. Enable it
  in Displays when colour detection is running. It is initially off so the
  sensor-only view does not wait for a behaviour node.

The IMU plugin is an explicit package dependency and is installed by rosdep.
Image subscriptions use reliable QoS to avoid losing large RGB messages. On a
VM, software rendering may lower the real-time factor and wall-clock image rate.
The existing 640 x 480 / 30 Hz camera configuration is preserved.

## Verify a session

In a second terminal, source ROS and the workspace with the same ROS domain and
Gazebo partition as the launch terminal:

```bash
ros2 topic echo /camera/camera_info --once
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 run tf2_ros tf2_echo odom camera_optical_link
```

Expect a 640 x 480 calibration, 181 LiDAR rays and IMU data with `imu_link` as its
frame. The camera image uses `camera_optical_link`; the scan uses `lidar_link`.
RViz should report OK for the robot, LiDAR, camera and IMU displays once data
arrives. The source-image bearing protocol and walking timings are described
in the [control-node guide](../porter_control/README.md).
