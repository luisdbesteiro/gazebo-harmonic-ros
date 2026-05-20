"""Run the ONNX locomotion policy and publish joint position targets."""

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from .constants import ACTION_DIM, ACTION_SCALE, DEFAULT_JOINT_POS, JOINT_ORDER, OBSERVATION_DIM

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover - depends on runtime environment
    ort = None


def default_onnx_path() -> str:
    """Return the repo-local default ONNX path when available."""
    candidate_suffix = Path(
        'workspace/results/raw/2025-tfg-diego-lopez/pruebas/G1/2026-01-26_10-42-02.onnx'
    )
    # Walk up from the installed module so this still works after colcon install.
    for parent in Path(__file__).resolve().parents:
        candidate = parent / candidate_suffix
        if candidate.exists():
            return str(candidate)
    return ''


class PolicyInference(Node):
    """Run inference from obs[99] and publish cmd_pos targets plus last_action."""

    def __init__(self) -> None:
        super().__init__('policy_inference')

        self.declare_parameter('observation_topic', '/g1/observation')
        self.declare_parameter('last_action_topic', '/g1/last_action')
        self.declare_parameter('cmd_pos_prefix', '/g1/cmd_pos')
        self.declare_parameter('onnx_model_path', default_onnx_path())
        self.declare_parameter('clip_action', False)
        self.declare_parameter('action_clip_min', -100.0)
        self.declare_parameter('action_clip_max', 100.0)

        observation_topic = self.get_parameter('observation_topic').get_parameter_value().string_value
        last_action_topic = self.get_parameter('last_action_topic').get_parameter_value().string_value
        cmd_pos_prefix = self.get_parameter('cmd_pos_prefix').get_parameter_value().string_value.rstrip('/')
        onnx_model_path = self.get_parameter('onnx_model_path').get_parameter_value().string_value
        self._clip_action = self.get_parameter('clip_action').get_parameter_value().bool_value
        self._action_clip_min = (
            self.get_parameter('action_clip_min').get_parameter_value().double_value
        )
        self._action_clip_max = (
            self.get_parameter('action_clip_max').get_parameter_value().double_value
        )

        self._session, self._input_name, self._output_name = self._load_session(onnx_model_path)
        self._last_observation: Optional[np.ndarray] = None

        self._last_action_publisher = self.create_publisher(Float64MultiArray, last_action_topic, 10)
        self._joint_publishers = {
            joint_name: self.create_publisher(Float64, f'{cmd_pos_prefix}/{joint_name}', 10)
            for joint_name in JOINT_ORDER
        }

        self.create_subscription(
            Float64MultiArray,
            observation_topic,
            self._on_observation,
            10,
        )

        self._default_joint_pos = np.asarray(DEFAULT_JOINT_POS, dtype=np.float32)
        self._action_scale = np.asarray(ACTION_SCALE, dtype=np.float32)

        self.get_logger().info(
            'policy_inference ready. '
            f'Loaded ONNX model from {onnx_model_path} and will publish cmd_pos for {len(JOINT_ORDER)} joints.'
        )

    def _load_session(self, onnx_model_path: str) -> tuple['ort.InferenceSession', str, str]:
        if ort is None:
            raise RuntimeError(
                'onnxruntime is not available. Add it to the Docker image before running policy_inference.'
            )

        if not onnx_model_path:
            raise RuntimeError(
                'No ONNX model path configured. Set the onnx_model_path parameter for policy_inference.'
            )

        model_path = Path(onnx_model_path)
        if not model_path.exists():
            raise RuntimeError(f'ONNX model not found: {model_path}')

        session = ort.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
        input_name = session.get_inputs()[0].name
        output_name = session.get_outputs()[0].name
        return session, input_name, output_name

    def _on_observation(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != OBSERVATION_DIM:
            self.get_logger().warning(
                f'Ignoring observation with size {len(msg.data)}. Expected {OBSERVATION_DIM}.'
            )
            return

        self._last_observation = np.asarray(msg.data, dtype=np.float32)
        # The observation is already published from simulation clock ticks, so inference
        # can run directly on each new message without using wall-time timers.
        self._run_inference()

    def _run_inference(self) -> None:
        if self._last_observation is None:
            return

        # The ONNX policy expects a single batch item with shape [1, 99].
        raw_action = self._session.run(
            [self._output_name],
            {self._input_name: self._last_observation[None, :]},
        )[0][0].astype(np.float32)

        if raw_action.shape != (ACTION_DIM,):
            self.get_logger().error(
                f'ONNX output has shape {raw_action.shape}; expected ({ACTION_DIM},).'
            )
            return

        if self._clip_action:
            raw_action = np.clip(raw_action, self._action_clip_min, self._action_clip_max)

        # Mirror MuJoCo: last_action stores raw policy output, while cmd_pos uses scaled offsets.
        joint_targets = self._default_joint_pos + raw_action * self._action_scale

        last_action_msg = Float64MultiArray()
        last_action_msg.data = raw_action.astype(np.float64).tolist()
        self._last_action_publisher.publish(last_action_msg)

        # Publish in the same joint order used to build the observation and action vectors.
        for joint_name, joint_target in zip(JOINT_ORDER, joint_targets):
            joint_msg = Float64()
            joint_msg.data = float(joint_target)
            self._joint_publishers[joint_name].publish(joint_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PolicyInference()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
