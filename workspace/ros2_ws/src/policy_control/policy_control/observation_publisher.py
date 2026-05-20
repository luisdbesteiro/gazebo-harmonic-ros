"""Publish the policy observation vector for the G1 robot."""

from math import sqrt
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from .constants import ACTION_DIM, COMMAND_DIM, DEFAULT_JOINT_POS, JOINT_ORDER, OBSERVATION_DIM


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    """Convert a quaternion into a 3x3 rotation matrix."""
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def project_gravity_from_quaternion(x: float, y: float, z: float, w: float) -> list[float]:
    """Project world gravity into the pelvis frame."""
    rot = quaternion_to_rotation_matrix(x, y, z, w)
    gravity_world = [0.0, 0.0, -1.0]
    # Match the MuJoCo observation, which uses gravity expressed in pelvis coordinates.
    return [
        rot[0][0] * gravity_world[0] + rot[1][0] * gravity_world[1] + rot[2][0] * gravity_world[2],
        rot[0][1] * gravity_world[0] + rot[1][1] * gravity_world[1] + rot[2][1] * gravity_world[2],
        rot[0][2] * gravity_world[0] + rot[1][2] * gravity_world[1] + rot[2][2] * gravity_world[2],
    ]


class ObservationPublisher(Node):
    """Aggregate robot state into the 99D observation expected by the policy."""

    def __init__(self) -> None:
        super().__init__('observation_publisher')

        self.declare_parameter('joint_state_topic', '/g1/joint_states')
        self.declare_parameter('pelvis_imu_topic', '/g1/imu/pelvis')
        self.declare_parameter('pelvis_odometry_topic', '/g1/pelvis/odometry')
        self.declare_parameter('pelvis_twist_topic', '/g1/pelvis_twist')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('last_action_topic', '/g1/last_action')
        self.declare_parameter('observation_topic', '/g1/observation')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('publish_rate_hz', 50.0)

        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        pelvis_imu_topic = self.get_parameter('pelvis_imu_topic').get_parameter_value().string_value
        pelvis_odometry_topic = self.get_parameter('pelvis_odometry_topic').get_parameter_value().string_value
        pelvis_twist_topic = self.get_parameter('pelvis_twist_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        last_action_topic = self.get_parameter('last_action_topic').get_parameter_value().string_value
        observation_topic = self.get_parameter('observation_topic').get_parameter_value().string_value
        clock_topic = self.get_parameter('clock_topic').get_parameter_value().string_value
        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self._joint_state: Optional[JointState] = None
        self._imu: Optional[Imu] = None
        self._pelvis_odometry: Optional[Odometry] = None
        self._pelvis_twist: Optional[TwistStamped] = None
        self._command = [0.0] * COMMAND_DIM
        self._last_action = [0.0] * ACTION_DIM
        self._missing_linear_velocity_warned = False
        self._last_publish_time_ns: Optional[int] = None
        self._publish_period_ns = int(1e9 / publish_rate_hz) if publish_rate_hz > 0.0 else 20_000_000

        self._observation_publisher = self.create_publisher(Float64MultiArray, observation_topic, 10)

        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)
        self.create_subscription(Imu, pelvis_imu_topic, self._on_imu, 10)
        self.create_subscription(Odometry, pelvis_odometry_topic, self._on_pelvis_odometry, 10)
        self.create_subscription(TwistStamped, pelvis_twist_topic, self._on_pelvis_twist, 10)
        self.create_subscription(Twist, command_topic, self._on_command, 10)
        self.create_subscription(Float64MultiArray, last_action_topic, self._on_last_action, 10)
        self.create_subscription(Clock, clock_topic, self._on_clock, 10)

        self.get_logger().info(
            'observation_publisher ready. '
            'It publishes obs[99] in MuJoCo joint order on simulation clock ticks and uses '
            f'pelvis linear velocity from {pelvis_odometry_topic} (fallback: {pelvis_twist_topic}).'
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _on_imu(self, msg: Imu) -> None:
        self._imu = msg

    def _on_pelvis_odometry(self, msg: Odometry) -> None:
        self._pelvis_odometry = msg

    def _on_pelvis_twist(self, msg: TwistStamped) -> None:
        self._pelvis_twist = msg

    def _on_command(self, msg: Twist) -> None:
        # The policy command is [vx, vy, wz], not the full Twist message.
        self._command = [
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        ]

    def _on_last_action(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != ACTION_DIM:
            self.get_logger().warning(
                f'Ignoring last_action with size {len(msg.data)}. Expected {ACTION_DIM}.'
            )
            return
        self._last_action = list(msg.data)

    def _on_clock(self, msg: Clock) -> None:
        current_time_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self._last_publish_time_ns is not None:
            if current_time_ns <= self._last_publish_time_ns:
                return
            if current_time_ns - self._last_publish_time_ns < self._publish_period_ns:
                return

        self._last_publish_time_ns = current_time_ns
        self._publish_observation()

    def _ordered_joint_values(self) -> Optional[tuple[list[float], list[float]]]:
        if self._joint_state is None:
            return None

        names = list(self._joint_state.name)
        pos = list(self._joint_state.position)
        vel = list(self._joint_state.velocity)

        if len(pos) != len(names):
            self.get_logger().warning('JointState position/name size mismatch.')
            return None

        if len(vel) != len(names):
            vel = [0.0] * len(names)

        position_map = {name: value for name, value in zip(names, pos)}
        velocity_map = {name: value for name, value in zip(names, vel)}

        missing = [name for name in JOINT_ORDER if name not in position_map]
        if missing:
            self.get_logger().warning(f'Missing joints in JointState: {missing}')
            return None

        # Reorder Gazebo joint states into the exact MuJoCo / policy joint order.
        ordered_pos = [position_map[name] for name in JOINT_ORDER]
        ordered_vel = [velocity_map.get(name, 0.0) for name in JOINT_ORDER]
        return ordered_pos, ordered_vel

    def _publish_observation(self) -> None:
        if self._imu is None or self._joint_state is None:
            return

        ordered = self._ordered_joint_values()
        if ordered is None:
            return

        ordered_pos, ordered_vel = ordered
        # MuJoCo feeds joint positions relative to the default knees-bent pose.
        qpos_rel = [pos - default for pos, default in zip(ordered_pos, DEFAULT_JOINT_POS)]

        if self._pelvis_odometry is not None:
            linear_velocity = [
                self._pelvis_odometry.twist.twist.linear.x,
                self._pelvis_odometry.twist.twist.linear.y,
                self._pelvis_odometry.twist.twist.linear.z,
            ]
        elif self._pelvis_twist is not None:
            linear_velocity = [
                self._pelvis_twist.twist.linear.x,
                self._pelvis_twist.twist.linear.y,
                self._pelvis_twist.twist.linear.z,
            ]
        else:
            if not self._missing_linear_velocity_warned:
                self.get_logger().warning(
                    'No pelvis linear velocity received yet. The first 3 observation entries will stay at zero. '
                    'Bridge /g1/pelvis/odometry or publish /g1/pelvis_twist to provide '
                    'MuJoCo-compatible linear velocity.'
                )
                self._missing_linear_velocity_warned = True
            linear_velocity = [0.0, 0.0, 0.0]

        angular_velocity = [
            self._imu.angular_velocity.x,
            self._imu.angular_velocity.y,
            self._imu.angular_velocity.z,
        ]

        gravity = project_gravity_from_quaternion(
            self._imu.orientation.x,
            self._imu.orientation.y,
            self._imu.orientation.z,
            self._imu.orientation.w,
        )

        # Keep the 99D layout identical to the MuJoCo rollout code.
        observation = (
            linear_velocity
            + angular_velocity
            + gravity
            + qpos_rel
            + ordered_vel
            + self._last_action
            + self._command
        )

        if len(observation) != OBSERVATION_DIM:
            self.get_logger().error(
                f'Observation has size {len(observation)} instead of {OBSERVATION_DIM}.'
            )
            return

        msg = Float64MultiArray()
        msg.data = observation
        self._observation_publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ObservationPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
