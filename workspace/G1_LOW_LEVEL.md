# Unitree G1 low-level control in Gazebo Harmonic

This setup uses the official Unitree `g1_29dof_rev_1_0.urdf` as source data and
adds a Gazebo Harmonic SDF variant for isolated joint-position experiments.

For joint-velocity experiments, use the separate setup documented in
`/workspace/G1_LOW_LEVEL_VELOCITY.md`.

The official description remains in:

```bash
/workspace/models/g1_description
```

The Gazebo-ready low-level model is:

```bash
/workspace/models/g1_29dof_low_level_control/model.sdf
```

## What is exposed to ROS 2

Gazebo publishes all 29 revolute joints on:

```bash
/g1/joint_states
```

Each joint accepts a ROS 2 `std_msgs/msg/Float64` position command in radians:

```bash
/g1/cmd_pos/<joint_name>
```

Example:

```bash
/g1/cmd_pos/left_shoulder_pitch_joint
```

## Start Gazebo

From the project root on the host:

```bash
docker compose run --rm gazebo_harmonic_ros2
```

Inside the container:

```bash
gz sim /workspace/worlds/g1_low_level.world.sdf
```

For server-only testing:

```bash
gz sim -s -r /workspace/worlds/g1_low_level.world.sdf
```

## Start the ROS 2 bridge

In another container shell:

```bash
docker compose exec gazebo_harmonic_ros2 bash
```

Then run:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/g1_bridge.yaml
```

## Move one joint

In another container shell:

```bash
ros2 topic pub --once /g1/cmd_pos/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.5}"
```

Return it near zero:

```bash
ros2 topic pub --once /g1/cmd_pos/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.0}"
```

Read the current joint positions:

```bash
ros2 topic echo /g1/joint_states
```

## Joint list

```text
left_hip_pitch_joint
left_hip_roll_joint
left_hip_yaw_joint
left_knee_joint
left_ankle_pitch_joint
left_ankle_roll_joint
right_hip_pitch_joint
right_hip_roll_joint
right_hip_yaw_joint
right_knee_joint
right_ankle_pitch_joint
right_ankle_roll_joint
waist_yaw_joint
waist_roll_joint
waist_pitch_joint
left_shoulder_pitch_joint
left_shoulder_roll_joint
left_shoulder_yaw_joint
left_elbow_joint
left_wrist_roll_joint
left_wrist_pitch_joint
left_wrist_yaw_joint
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_joint
right_wrist_roll_joint
right_wrist_pitch_joint
right_wrist_yaw_joint
```

## Current modeling choice

This model fixes the pelvis to the Gazebo world with a fixed joint. Gravity is
enabled, but the base cannot drift through the scene, so you can test individual
joints without also solving whole-body balance. A later dynamic walking or
balance setup should use a separate free-base model variant with a proper
controller.
