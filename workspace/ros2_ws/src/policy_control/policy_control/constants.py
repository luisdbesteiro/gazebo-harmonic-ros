"""Constants for the G1 locomotion policy interface."""

JOINT_ORDER = [
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_hip_yaw_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_hip_yaw_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'waist_yaw_joint',
    'waist_roll_joint',
    'waist_pitch_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
]

# KNEES_BENT_KEYFRAME from the MuJoCo reference rollout.
DEFAULT_JOINT_POS = [
    -0.312,  # left_hip_pitch_joint
    0.0,     # left_hip_roll_joint
    0.0,     # left_hip_yaw_joint
    0.669,   # left_knee_joint
    -0.363,  # left_ankle_pitch_joint
    0.0,     # left_ankle_roll_joint
    -0.312,  # right_hip_pitch_joint
    0.0,     # right_hip_roll_joint
    0.0,     # right_hip_yaw_joint
    0.669,   # right_knee_joint
    -0.363,  # right_ankle_pitch_joint
    0.0,     # right_ankle_roll_joint
    0.0,     # waist_yaw_joint
    0.0,     # waist_roll_joint
    0.0,     # waist_pitch_joint
    0.2,     # left_shoulder_pitch_joint
    0.2,     # left_shoulder_roll_joint
    0.0,     # left_shoulder_yaw_joint
    0.6,     # left_elbow_joint
    0.0,     # left_wrist_roll_joint
    0.0,     # left_wrist_pitch_joint
    0.0,     # left_wrist_yaw_joint
    0.2,     # right_shoulder_pitch_joint
    -0.2,    # right_shoulder_roll_joint
    0.0,     # right_shoulder_yaw_joint
    0.6,     # right_elbow_joint
    0.0,     # right_wrist_roll_joint
    0.0,     # right_wrist_pitch_joint
    0.0,     # right_wrist_yaw_joint
]

ACTION_SCALE = [
    0.49869924033661583,  # left_hip_pitch_joint
    0.23391105917489544,  # left_hip_roll_joint
    0.49869924033661583,  # left_hip_yaw_joint
    0.23391105917489544,  # left_knee_joint
    0.36295718076906824,  # left_ankle_pitch_joint
    0.36295718076906824,  # left_ankle_roll_joint
    0.49869924033661583,  # right_hip_pitch_joint
    0.23391105917489544,  # right_hip_roll_joint
    0.49869924033661583,  # right_hip_yaw_joint
    0.23391105917489544,  # right_knee_joint
    0.36295718076906824,  # right_ankle_pitch_joint
    0.36295718076906824,  # right_ankle_roll_joint
    0.49869924033661583,  # waist_yaw_joint
    0.36295718076906824,  # waist_roll_joint
    0.36295718076906824,  # waist_pitch_joint
    0.36295718076906824,  # left_shoulder_pitch_joint
    0.36295718076906824,  # left_shoulder_roll_joint
    0.36295718076906824,  # left_shoulder_yaw_joint
    0.36295718076906824,  # left_elbow_joint
    0.36295718076906824,  # left_wrist_roll_joint
    46.56304395328023,    # left_wrist_pitch_joint
    46.56304395328023,    # left_wrist_yaw_joint
    0.36295718076906824,  # right_shoulder_pitch_joint
    0.36295718076906824,  # right_shoulder_roll_joint
    0.36295718076906824,  # right_shoulder_yaw_joint
    0.36295718076906824,  # right_elbow_joint
    0.36295718076906824,  # right_wrist_roll_joint
    46.56304395328023,    # right_wrist_pitch_joint
    46.56304395328023,    # right_wrist_yaw_joint
]

OBSERVATION_DIM = 99
ACTION_DIM = 29
COMMAND_DIM = 3
