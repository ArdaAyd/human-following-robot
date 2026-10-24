# Porter camera

The fixed RGB camera looks forward from `(sensor_x, 0, camera_height)` in
`base_footprint`: currently `(0.41, 0, 1.00)` m. It shares the LiDAR's X/Y
position. A 0.5 kg post and arm support the 0.1 kg camera; their collision
geometry and box inertias are included. The post stands at `mast_post_x`, behind
the LiDAR's forward scan plane. Placement uses the chassis properties rather
than separate mount coordinates.

`camera_link` uses the robot convention (+X forward, +Y left, +Z up).
`camera_optical_link` is rotated by `(-pi/2, 0, -pi/2)` and uses +Z forward,
+X right and +Y down. Image and calibration messages both name this optical
frame. The Gazebo sensor itself stays aligned with `camera_link`, so the optical
TF rotation does not turn the rendered view sideways.

| Setting | Value |
|---|---|
| RGB image | 640 x 480, `rgb8` |
| Horizontal field of view | 80 degrees |
| Update rate | 30 Hz in simulation time |
| Clipping range | 0.05-30 m |
| Image topic | `/camera/image_raw` (`sensor_msgs/msg/Image`) |
| Calibration topic | `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`) |
| Message frame | `camera_optical_link` |

Both topics use the single shared `porter_gazebo/config/bridge.yaml`.
The project world already loads the Gazebo Sensors system with Ogre2.

## Quick start

After building and sourcing the workspace, open the complete stationary sensor
view with:

```bash
ros2 launch porter_gazebo bringup.launch.py walker:=false
```

The [bringup guide](../porter_gazebo/README.md) describes all launch options.
The manual steps below are useful when inspecting each component separately.

## Manual camera demo

Build from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
xacro "$(ros2 pkg prefix porter_description)/share/porter_description/urdf/porter.urdf.xacro" > /tmp/porter-camera.urdf
```

Source ROS and `install/setup.bash` in each subsequent terminal and use the same
ROS domain and Gazebo partition throughout. Start the world in terminal 1:

```bash
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix porter_gazebo)/share/porter_gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r "$(ros2 pkg prefix porter_gazebo)/share/porter_gazebo/worlds/porter_world.sdf"
```

In terminal 2, insert the robot behind the stationary person at the origin:

```bash
ros2 run ros_gz_sim create -world porter_world -name porter \
  -file /tmp/porter-camera.urdf -x -1.0 -y 0 -z 0.01
```

In terminal 3, publish the robot's TF tree:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
parameters = {'robot_state_publisher': {'ros__parameters': {
    'use_sim_time': True,
    'robot_description': Path('/tmp/porter-camera.urdf').read_text(),
}}}
Path('/tmp/porter-camera-params.yaml').write_text(yaml.safe_dump(parameters))
PY
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  --params-file /tmp/porter-camera-params.yaml
```

In terminal 4, start the shared bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$(ros2 pkg prefix porter_gazebo)/share/porter_gazebo/config/bridge.yaml"
```

In terminal 5, open the live camera image:

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

The orange vest should appear near the horizontal centre of the image. Keep
`person_walker` stopped during this stationary check. The camera alone does not
publish a target bearing or drive the robot.

## Verification

Inspect the image rate and calibration from another sourced terminal:

```bash
ros2 topic hz /camera/image_raw --qos-reliability reliable
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo base_footprint camera_optical_link
```

Expect 640 x 480 images and matching `camera_optical_link` headers on both camera
topics. At 80 degrees horizontal FOV, `fx` and `fy` are about 381.36 pixels and
the principal point is near `(320, 240)`. A person to the robot's left appears
left in the image; a person to its right appears right. The sensor centre is at
`(0.41, 0, 1.0)` m relative to `base_footprint`.

The rate is configured in simulation time. On a VM using software rendering,
`ros2 topic hz` measures a lower wall-clock rate when Gazebo's real-time factor
falls below one. Compare message timestamps as well as the real-time factor.
Use reliable image subscriptions when checking the full publication rate;
best-effort subscribers can drop these large RGB messages and underreport it.
The QEMU/software-rendering check delivered about 30 Hz by message timestamps
and roughly 13-19 frames/s in wall time at 640 x 480. Check real-time 30 Hz separately on
the presentation machine with GPU acceleration.
