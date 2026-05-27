#pragma once

#include <array>
#include <cstddef>
#include <string_view>

namespace policy_control
{

inline constexpr std::size_t kActionDim = 29;
inline constexpr std::size_t kCommandDim = 3;
inline constexpr std::size_t kObservationDim = 99;

// Keep this order identical to the MuJoCo policy order and the Python constants.
inline constexpr std::array<std::string_view, kActionDim> kJointOrder = {
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
};

// KNEES_BENT_KEYFRAME from the MuJoCo reference rollout.
inline constexpr std::array<double, kActionDim> kDefaultJointPos = {
  -0.312,
  0.0,
  0.0,
  0.669,
  -0.363,
  0.0,
  -0.312,
  0.0,
  0.0,
  0.669,
  -0.363,
  0.0,
  0.0,
  0.0,
  0.0,
  0.2,
  0.2,
  0.0,
  0.6,
  0.0,
  0.0,
  0.0,
  0.2,
  -0.2,
  0.0,
  0.6,
  0.0,
  0.0,
  0.0,
};

// Per-joint scale applied to the raw network action before publishing cmd_pos.
inline constexpr std::array<double, kActionDim> kActionScale = {
  0.49869924033661583,
  0.23391105917489544,
  0.49869924033661583,
  0.23391105917489544,
  0.36295718076906824,
  0.36295718076906824,
  0.49869924033661583,
  0.23391105917489544,
  0.49869924033661583,
  0.23391105917489544,
  0.36295718076906824,
  0.36295718076906824,
  0.49869924033661583,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
  0.36295718076906824,
};

}  // namespace policy_control
