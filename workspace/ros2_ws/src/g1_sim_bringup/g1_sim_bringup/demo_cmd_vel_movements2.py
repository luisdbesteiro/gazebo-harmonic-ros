import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_TOPICS = {
    'waist_yaw_joint': '/g1/cmd_vel/waist_yaw_joint',
    'waist_roll_joint': '/g1/cmd_vel/waist_roll_joint',
    'waist_pitch_joint': '/g1/cmd_vel/waist_pitch_joint',
    'left_shoulder_roll_joint': '/g1/cmd_vel/left_shoulder_roll_joint',
    'left_shoulder_pitch_joint': '/g1/cmd_vel/left_shoulder_pitch_joint',
    'left_shoulder_yaw_joint': '/g1/cmd_vel/left_shoulder_yaw_joint',
    'right_shoulder_pitch_joint': '/g1/cmd_vel/right_shoulder_pitch_joint',
    'left_elbow_joint': '/g1/cmd_vel/left_elbow_joint',
    'right_elbow_joint': '/g1/cmd_vel/right_elbow_joint',
    'left_hip_pitch_joint': '/g1/cmd_vel/left_hip_pitch_joint',
    'right_hip_pitch_joint': '/g1/cmd_vel/right_hip_pitch_joint',
    'left_knee_joint': '/g1/cmd_vel/left_knee_joint',
    'right_knee_joint': '/g1/cmd_vel/right_knee_joint',
    'left_ankle_pitch_joint':'/g1/cmd_vel/left_ankle_pitch_joint',
    'right_ankle_pitch_joint':'/g1/cmd_vel/right_ankle_pitch_joint',
}


PHASES = [
    {
        'name': 'left_arm_up',
        'duration': 2.0,
        'commands': {
            'left_shoulder_roll_joint': 0.9,
            'left_shoulder_yaw_joint': 0.7,
        },
    },
    {
        'name': 'wave_1',
        'duration': 1.0,
        'commands': {
            'left_elbow_joint': 0.50,
        },
    },
    {
        'name': 'wave_2',
        'duration': 2.0,
        'commands': {
            'left_elbow_joint': -0.50,
        },
    },
    {
        'name': 'wave_3',
        'duration': 2.0,
        'commands': {
            'left_elbow_joint': 0.50,
        },
    },
    {
        'name': 'wave_4',
        'duration': 1.0,
        'commands': {
            'left_elbow_joint': -0.50,
        },
    },
    {
        'name': 'recover_arms',
        'duration': 2.0,
        'commands': {
            'left_shoulder_roll_joint': -0.9,
            'left_shoulder_yaw_joint': -0.7,
        },
    },
    {
        'name': 'step_up',
        'duration': 4.0,
        'commands': {
            'left_hip_pitch_joint': -0.30,
            'left_knee_joint': 0.45,
            'left_ankle_pitch_joint': -0.15,
            'right_hip_pitch_joint': 0.07,
            'right_ankle_pitch_joint': 0.15,

            'waist_pitch_joint': 0.06,
            'waist_yaw_joint': 0.05,
            
            'left_elbow_joint': 0.25,
            'left_shoulder_pitch_joint': 0.20,
            'right_elbow_joint': -0.15,
            'right_shoulder_pitch_joint': -0.25,
        },
    },
    {
        'name': 'recover_step',
        'duration': 4.0,
        'commands': {
            'left_hip_pitch_joint': 0.30,
            'left_knee_joint': -0.45,
            'left_ankle_pitch_joint': 0.15,
            'right_hip_pitch_joint': -0.07,
            'right_ankle_pitch_joint': -0.15,

            'waist_pitch_joint': -0.06,
            'waist_yaw_joint': -0.05,
            
            'left_elbow_joint': -0.25,
            'left_shoulder_pitch_joint': -0.20,
            'right_elbow_joint': 0.15,
            'right_shoulder_pitch_joint': 0.25,
        },
    },
    {
        'name': 'stop',
        'duration': 2.0,
        'commands': {},
    },
]


class DemoCmdVelMovementsNode(Node):
    def __init__(self) -> None:
        super().__init__('demo_cmd_vel_movements')
        self._publishers = {
            joint_name: self.create_publisher(Float64, topic_name, 10)
            for joint_name, topic_name in JOINT_TOPICS.items()
        }
        self._start_time = time.monotonic()
        self._last_phase_name = None
        self._timer = self.create_timer(0.05, self._on_timer)

        total_cycle_time = sum(phase['duration'] for phase in PHASES)
        self.get_logger().info(
            'Publishing time-based joint velocity commands on /g1/cmd_vel/* '
            f'with a {total_cycle_time:.1f}s repeating cycle.'
        )

    def _on_timer(self) -> None:
        elapsed = time.monotonic() - self._start_time
        phase_name, commands = self._commands_for_elapsed_time(elapsed)

        if phase_name != self._last_phase_name:
            self._last_phase_name = phase_name
            self.get_logger().info(
                f'Entering phase "{phase_name}" at t={elapsed:.2f}s.'
            )

        for joint_name, publisher in self._publishers.items():
            msg = Float64()
            msg.data = commands.get(joint_name, 0.0)
            publisher.publish(msg)

    def _commands_for_elapsed_time(self, elapsed: float):
        cycle_duration = sum(phase['duration'] for phase in PHASES)
        phase_time = elapsed % cycle_duration

        accumulated = 0.0
        for phase in PHASES:
            accumulated += phase['duration']
            if phase_time < accumulated:
                return phase['name'], phase['commands']

        final_phase = PHASES[-1]
        return final_phase['name'], final_phase['commands']


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoCmdVelMovementsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
