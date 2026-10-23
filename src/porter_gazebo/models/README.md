# Person model

`person.sdf` is a dynamic, collision-bearing target for Porter. The model origin
is at ground level and +X is its forward direction. It has a 1.7 m tall,
0.35 m diameter cylindrical body and an orange vest band from 0.925 to 1.375 m.
Both visible parts have collision geometry. The body reaches the ground so a
horizontal LiDAR scan at 0.25 m can detect it.

The single rigid link has a mass of 60 kg and the analytical inertia of the body
cylinder; the thin vest is treated as a covering with negligible added mass.
The body contact is frictionless so prescribed sliding motion does not tip the
cylinder. The model is deliberately simple and has no walking animation.

## Standalone check

From the workspace root, build and start the bundled empty world:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
gz sdf --check src/porter_gazebo/models/person.sdf
gz sim -r empty.sdf
```

In another terminal, from the same workspace root, insert the model:

```bash
source /opt/ros/lyrical/setup.bash
gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: \"$PWD/src/porter_gazebo/models/person.sdf\", name: \"person\""
```

Check that the orange band is visible. Enable collision visualisation in the
Gazebo GUI and check the body and vest shapes. For a physical collision check,
drop a small box centred above the person: it should rest on the body's top at
1.7 m instead of falling through it.

The `VelocityControl` plugin receives `gz.msgs.Twist` on `/person/cmd_vel`.
Test forward motion and then send an explicit stop:

```bash
gz topic -t /person/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.3}'
gz topic -t /person/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0} angular: {z: 0}'
```

The plugin holds the most recent command, so stopping the publisher does not
stop the model. Always send a zero command after a motion check. The commands
above use Gazebo Transport directly. To run the ROS bridge and the fixed walking
scenario, follow [`porter_control/README.md`](../../porter_control/README.md).
The ROS scenario uses `porter_world.sdf`, not this standalone empty world,
because the shared bridge maps `/world/porter_world/clock`. The project world
already includes the person; do not spawn a second copy there. The longer route
climbs the ramp on its final southbound leg and finishes on the deck.

## World integration

An SDF world can include the file relative to its own location:

```xml
<include>
  <uri>../models/person.sdf</uri>
  <name>person</name>
  <pose>2 0 0 0 0 0</pose>
</include>
```

The existing package installation rule installs `models/`, including this file.
