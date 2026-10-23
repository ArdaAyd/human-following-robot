# Person walker

`person_walker` publishes `geometry_msgs/msg/Twist` on `/person/cmd_vel` at
20 Hz. The supplied configuration uses Gazebo's ROS simulation clock. The
command heartbeat continues during a pause, but elapsed scenario time freezes.
The sequence waits for a nonzero simulation clock and a command subscriber
(normally the ROS-to-Gazebo bridge), then starts its delay. Rewinding the clock
restarts the sequence.

## Default scenario

Times are relative to readiness, not to world startup. All motion is expressed
in the person's body frame: +X forward, +Y left, +Z yaw counterclockwise.

| Time (s) | Phase | Command |
|---|---|---|
| 0-2 | Start delay | Stop |
| 2-10 | Straight | 0.3 m/s forward (2.4 m) |
| 10-13 | Turn | 0.5236 rad/s left (90 degrees) |
| 13-23 | Ramp leg | 0.3 m/s forward (3 m) |
| 23-26 | Sudden sidestep | 0.3 m/s left (0.9 m) |
| 26 onward | Stopped | Repeated zero velocity |

The ramp leg is a timed forward command, not terrain detection. The world owner
must align a ramp with that leg and validate physical traversal. Likewise,
leaving Porter's camera view during the sidestep must be checked after robot
integration. The sequence runs once; restart the node to repeat from the
person's current pose, or reset the world and restart for the same starting pose.

## Run with the person model

Build from the workspace root and load the environment:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Start Gazebo and insert `person.sdf` using the instructions in
[`porter_gazebo/models/README.md`](../porter_gazebo/models/README.md).
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
`ros2 topic hz /person/cmd_vel`. The model should walk, turn, move forward,
sidestep, and stop. Keep the node running to continue publishing the final stop.
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

## Tests

From a sourced workspace, run the walker tests with:

```bash
python3 -m pytest src/porter_control/test/test_person_walker*.py
```

These cover phase boundaries, skipped phases, invalid settings, receiver
readiness, paused simulation time, clock rewind, and final stop delivery.
The ROS test remaps its clock and command topics to a private test prefix.
