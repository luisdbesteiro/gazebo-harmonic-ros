# Unitree G1 low-level velocity control in Gazebo Harmonic

This setup is a separate Gazebo model variant for joint-velocity experiments.
The original position-control setup is still available in `g1_low_level.world.sdf`.

## What is exposed to ROS 2

Gazebo publishes all 29 revolute joints on:

```bash
/g1/joint_states
```

Each joint accepts a ROS 2 `std_msgs/msg/Float64` velocity command in radians
per second:

```bash
/g1/cmd_vel/<joint_name>
```

Example:

```bash
/g1/cmd_vel/left_shoulder_pitch_joint
```

`geometry_msgs/msg/Twist` is not used for individual G1 actuators here. A
`Twist` message has 6 spatial velocity components; the G1 low-level actuator
interface has 29 independent joint velocities. For direct actuator control,
publish one `Float64` per joint.

## Start Gazebo

From the project root on the host:

```bash
docker compose run --rm gazebo_harmonic_ros2
```

Inside the container:

```bash
gz sim /workspace/worlds/g1_low_level_velocity.world.sdf
```

For server-only testing:

```bash
gz sim -s -r /workspace/worlds/g1_low_level_velocity.world.sdf
```

## Start the ROS 2 bridge

In another container shell:

```bash
docker compose exec gazebo_harmonic_ros2 bash
```

Then run:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/g1_velocity_bridge.yaml
```

## Move one joint by velocity

In another container shell:

```bash
ros2 topic pub --once /g1/cmd_vel/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.5}"
```

Stop it:

```bash
ros2 topic pub --once /g1/cmd_vel/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.0}"
```

Read the current joint states:

```bash
ros2 topic echo /g1/joint_states
```

## Current modeling choice

This model fixes the pelvis to the Gazebo world with a fixed joint. Gravity is
enabled, but the base cannot drift through the scene, so you can test individual
joints without also solving whole-body balance. A later dynamic walking or
balance setup should use a separate free-base model variant with a proper
whole-body controller.
