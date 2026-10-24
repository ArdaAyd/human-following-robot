# Porter — Human-Following Load Carrier

Porter is a differential-drive mobile robot that carries a payload and follows a
person wearing an orange safety vest at a distance of roughly one metre. It is
simulated in Gazebo and controlled through ROS 2.

The camera supplies the **bearing** to the person, the LiDAR supplies the
**range along that bearing**, and the IMU caps the forward speed on ramps. A
single behaviour node fuses the three and publishes `/cmd_vel`.

Midterm project for BIMU4101 Introduction to Mobile Robotics, 2026.

## Team

| Name | Student ID | Sensor responsibility |
|---|---|---|
| Arda AYDIN | 1306230121 | LiDAR |
| Ahmet Melih ÜSTÜNER | 1306230119 | Camera |

The IMU is the additional sensor and is a shared responsibility.

## Requirements

- ROS 2 Lyrical Luth
- Gazebo Jetty (gz-sim 10)
- Ubuntu 26.04

The project is developed and tested against these versions.

## Installation

```bash
git clone https://github.com/ArdaAyd/human-following-robot.git ~/porter_ws
cd ~/porter_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running

Start the world, robot, three-sensor RViz view and person scenario:

```bash
ros2 launch porter_gazebo bringup.launch.py
```

For a stationary camera check, add `walker:=false`. For sensor visualisation
without behaviour nodes, add `behaviour:=false`. See the [bringup guide](src/porter_gazebo/README.md)
for headless mode, startup/shutdown behaviour and the RViz display descriptions.

Automatic following remains off by default (`follow:=false`) while the controller
implementation is pending. The [colour detector guide](src/porter_control/README.md#colour-detector)
explains the bearing and debug mask, and the [person walker guide](src/porter_control/README.md#person-walker)
describes the 72-second route.

## Packages

| Package | Contents |
|---|---|
| `porter_description` | URDF/Xacro model and sensor definitions |
| `porter_gazebo` | World, person model, bridge configuration, launch file, RViz configuration |
| `porter_control` | Colour detection, following behaviour and demo scenario nodes |

## Topics

| Topic | Message type | Direction |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo → color_detector |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → visualisation |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → follow_controller |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo → follow_controller |
| `/target` | `geometry_msgs/msg/PointStamped` | color_detector → follow_controller |
| `/target_debug_image` | `sensor_msgs/msg/Image` | color_detector → display |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | follow_controller → DiffDrive |
| `/person/cmd_vel` | `geometry_msgs/msg/Twist` | person_walker → person model |

In `/target`, `point.x` is the bearing to the person in radians (positive to the
left) and `point.y` is the pixel area of the detected blob.

## Development workflow

`main` always builds and runs. Nothing is committed to it directly.

1. Branch off `main`, one branch per piece of work:

   ```bash
   git switch main
   git pull
   git switch -c feature/lidar-sensor
   ```

   Branch names: `feature/<what>` for new work, `fix/<what>` for corrections.

2. Commit in small, self-contained steps. Write the message in the imperative
   mood and say what the change does, not which files it touches:

   ```
   Add LiDAR sensor to the robot model
   Publish target bearing from the largest orange contour
   Fix ray index when the bearing is negative
   ```

   Avoid `update`, `fix stuff`, `wip` and commits that mix unrelated changes.

3. Push the branch and open a pull request:

   ```bash
   git push -u origin feature/lidar-sensor
   ```

4. The other member reviews the pull request and leaves comments. Merge once it
   is approved and the workspace builds.

5. Delete the branch after merging.

Each member owns a distinct set of files, so two people should never need to edit
the same file at the same time. If that does happen, say so before starting.

## Status

- [x] Workspace and package layout
- [x] Robot model (URDF/Xacro) and motion via `/cmd_vel`
- [x] LiDAR integration and visualisation
- [x] Camera integration and visualisation
- [x] IMU integration and visualisation
- [x] Gazebo world and person model
- [x] Single bringup launch file
- [x] `color_detector` — colour detection and bearing
- [ ] `follow_controller` — following behaviour
- [x] `person_walker` — demo scenario

## License

MIT
