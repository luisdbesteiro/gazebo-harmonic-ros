#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "policy_control/policy_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using rosgraph_msgs::msg::Clock;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Float64;
using std_msgs::msg::Float64MultiArray;

constexpr char kDefaultOnnxRelativePath[] = "results/2026-01-26_10-42-02.onnx";
constexpr char kDefaultOnnxWorkspacePath[] = "/workspace/results/2026-01-26_10-42-02.onnx";

// Prefer the mounted Docker workspace path, but also support running from a checked-out repo.
std::string default_onnx_path()
{
  namespace fs = std::filesystem;

  std::vector<fs::path> candidates;
  candidates.emplace_back(kDefaultOnnxWorkspacePath);

  std::error_code ec;
  fs::path current = fs::current_path(ec);
  while (!ec && !current.empty()) {
    candidates.emplace_back(current / kDefaultOnnxRelativePath);
    candidates.emplace_back(current / "workspace" / kDefaultOnnxRelativePath);

    if (current == current.parent_path()) {
      break;
    }
    current = current.parent_path();
  }

  for (const auto & candidate : candidates) {
    if (fs::exists(candidate, ec) && !ec) {
      return candidate.string();
    }
  }

  return kDefaultOnnxWorkspacePath;
}

std::array<double, 3> project_gravity_from_quaternion(double x, double y, double z, double w)
{
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm == 0.0) {
    return {0.0, 0.0, -1.0};
  }

  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;

  const std::array<std::array<double, 3>, 3> rot = {{
    {{1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)}},
    {{2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)}},
    {{2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)}},
  }};

  return {
    // The policy expects world gravity expressed in the pelvis frame.
    -rot[2][0],
    -rot[2][1],
    -rot[2][2],
  };
}

template <typename T, std::size_t N>
Float64MultiArray to_multi_array(const std::array<T, N> & values)
{
  Float64MultiArray msg;
  msg.data.reserve(values.size());
  for (const auto value : values) {
    msg.data.push_back(static_cast<double>(value));
  }
  return msg;
}

}  // namespace

namespace policy_control
{

class PolicyControllerCpp : public rclcpp::Node
{
public:
  PolicyControllerCpp()
  : Node("policy_controller_cpp"),
    ort_env_(ORT_LOGGING_LEVEL_WARNING, "policy_controller_cpp")
  {
    // This node merges observation_publisher and policy_inference into one control loop.
    const auto joint_state_topic = declare_parameter<std::string>("joint_state_topic", "/g1/joint_states");
    const auto pelvis_imu_topic = declare_parameter<std::string>("pelvis_imu_topic", "/g1/imu/pelvis");
    const auto pelvis_odometry_topic =
      declare_parameter<std::string>("pelvis_odometry_topic", "/g1/pelvis/odometry");
    const auto pelvis_twist_topic = declare_parameter<std::string>("pelvis_twist_topic", "/g1/pelvis_twist");
    const auto command_topic = declare_parameter<std::string>("command_topic", "/cmd_vel");
    const auto last_action_topic = declare_parameter<std::string>("last_action_topic", "/g1/last_action");
    const auto observation_topic = declare_parameter<std::string>("observation_topic", "/g1/observation");
    const auto cmd_pos_prefix =
      strip_trailing_slash(declare_parameter<std::string>("cmd_pos_prefix", "/g1/cmd_pos"));
    const auto onnx_model_path = declare_parameter<std::string>("onnx_model_path", default_onnx_path());
    const auto clock_topic = declare_parameter<std::string>("clock_topic", "/clock");
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);

    publish_observation_ = declare_parameter<bool>("publish_observation", false);
    clip_action_ = declare_parameter<bool>("clip_action", false);
    action_clip_min_ = declare_parameter<double>("action_clip_min", -100.0);
    action_clip_max_ = declare_parameter<double>("action_clip_max", 100.0);
    action_scale_factor_ = declare_parameter<double>("action_scale_factor", 1.0);
    upper_body_scale_factor_ = declare_parameter<double>("upper_body_scale_factor", 1.0);
    wrist_scale_factor_ = declare_parameter<double>("wrist_scale_factor", 1.0);
    hold_nominal_pose_duration_ns_ =
      static_cast<int64_t>(declare_parameter<double>("hold_nominal_pose_duration_s", 0.0) * 1e9);
    publish_period_ns_ = publish_rate_hz > 0.0 ?
      static_cast<int64_t>(1e9 / publish_rate_hz) :
      20'000'000;

    load_session(onnx_model_path);

    // Outputs mirror policy_inference: last_action plus one position command per joint.
    last_action_publisher_ = create_publisher<Float64MultiArray>(last_action_topic, 10);
    if (publish_observation_) {
      observation_publisher_ = create_publisher<Float64MultiArray>(observation_topic, 10);
    }

    for (const auto joint_name : kJointOrder) {
      joint_publishers_.emplace(
        std::string(joint_name),
        create_publisher<Float64>(cmd_pos_prefix + "/" + std::string(joint_name), 10));
    }

    // Sensor callbacks only cache the latest state; /clock triggers the actual control step.
    joint_state_sub_ = create_subscription<JointState>(
      joint_state_topic, 10, [this](JointState::SharedPtr msg) { joint_state_ = *msg; });
    imu_sub_ = create_subscription<Imu>(
      pelvis_imu_topic, 10, [this](Imu::SharedPtr msg) { imu_ = *msg; });
    pelvis_odometry_sub_ = create_subscription<Odometry>(
      pelvis_odometry_topic, 10, [this](Odometry::SharedPtr msg) { pelvis_odometry_ = *msg; });
    pelvis_twist_sub_ = create_subscription<TwistStamped>(
      pelvis_twist_topic, 10, [this](TwistStamped::SharedPtr msg) { pelvis_twist_ = *msg; });
    command_sub_ = create_subscription<Twist>(
      command_topic, 10, [this](Twist::SharedPtr msg) {
        command_ = {msg->linear.x, msg->linear.y, msg->angular.z};
      });
    clock_sub_ = create_subscription<Clock>(
      clock_topic, 10, [this](Clock::SharedPtr msg) { on_clock(*msg); });

    RCLCPP_INFO(
      get_logger(),
      "policy_controller_cpp ready. Loaded ONNX model from %s and will run observation + inference on %s.",
      onnx_model_path.c_str(),
      clock_topic.c_str());
  }

private:
  using OrderedJointValues =
    std::pair<std::array<double, kActionDim>, std::array<double, kActionDim>>;

  static std::string strip_trailing_slash(std::string value)
  {
    while (value.size() > 1 && value.back() == '/') {
      value.pop_back();
    }
    return value;
  }

  void load_session(const std::string & onnx_model_path)
  {
    if (onnx_model_path.empty()) {
      throw std::runtime_error("No ONNX model path configured. Set the onnx_model_path parameter.");
    }

    if (!std::filesystem::exists(onnx_model_path)) {
      throw std::runtime_error("ONNX model not found: " + onnx_model_path);
    }

    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = std::make_unique<Ort::Session>(ort_env_, onnx_model_path.c_str(), session_options_);

    // The exported policy is treated as a single-input, single-output ONNX graph.
    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = session_->GetInputNameAllocated(0, allocator);
    auto output_name = session_->GetOutputNameAllocated(0, allocator);
    input_name_ = input_name.get();
    output_name_ = output_name.get();
  }

  void on_clock(const Clock & msg)
  {
    const int64_t current_time_ns =
      static_cast<int64_t>(msg.clock.sec) * 1'000'000'000LL + static_cast<int64_t>(msg.clock.nanosec);

    if (last_publish_time_ns_.has_value()) {
      if (current_time_ns <= *last_publish_time_ns_) {
        return;
      }
      if (current_time_ns - *last_publish_time_ns_ < publish_period_ns_) {
        return;
      }
    }

    current_sim_time_ns_ = current_time_ns;
    last_publish_time_ns_ = current_time_ns;
    // Run only when simulation time advances enough for the configured policy rate.
    run_control_step();
  }

  std::optional<OrderedJointValues> ordered_joint_values()
  {
    if (!joint_state_.has_value()) {
      return std::nullopt;
    }

    const auto & names = joint_state_->name;
    const auto & positions = joint_state_->position;
    const auto & velocities = joint_state_->velocity;

    if (positions.size() != names.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "JointState position/name size mismatch.");
      return std::nullopt;
    }

    std::unordered_map<std::string, double> position_map;
    std::unordered_map<std::string, double> velocity_map;
    position_map.reserve(names.size());
    velocity_map.reserve(names.size());

    const bool velocities_valid = velocities.size() == names.size();
    for (std::size_t i = 0; i < names.size(); ++i) {
      position_map[names[i]] = positions[i];
      velocity_map[names[i]] = velocities_valid ? velocities[i] : 0.0;
    }

    std::vector<std::string> missing;
    for (const auto joint_name : kJointOrder) {
      if (position_map.find(std::string(joint_name)) == position_map.end()) {
        missing.push_back(std::string(joint_name));
      }
    }

    if (!missing.empty()) {
      std::ostringstream missing_stream;
      for (std::size_t i = 0; i < missing.size(); ++i) {
        if (i != 0) {
          missing_stream << ", ";
        }
        missing_stream << missing[i];
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Missing joints in JointState: %s",
        missing_stream.str().c_str());
      return std::nullopt;
    }

    std::array<double, kActionDim> ordered_pos{};
    std::array<double, kActionDim> ordered_vel{};
    // Reorder Gazebo JointState fields into the fixed policy action/observation order.
    for (std::size_t i = 0; i < kJointOrder.size(); ++i) {
      const auto joint_name = std::string(kJointOrder[i]);
      ordered_pos[i] = position_map[joint_name];
      ordered_vel[i] = velocity_map[joint_name];
    }

    return OrderedJointValues{ordered_pos, ordered_vel};
  }

  std::optional<std::array<float, kObservationDim>> build_observation()
  {
    if (!imu_.has_value() || !joint_state_.has_value()) {
      return std::nullopt;
    }

    const auto ordered = ordered_joint_values();
    if (!ordered.has_value()) {
      return std::nullopt;
    }

    std::array<double, 3> linear_velocity{};
    // MuJoCo provided pelvis linear velocity directly; Gazebo supplies it via odometry/bridge.
    if (pelvis_odometry_.has_value()) {
      linear_velocity = {
        pelvis_odometry_->twist.twist.linear.x,
        pelvis_odometry_->twist.twist.linear.y,
        pelvis_odometry_->twist.twist.linear.z,
      };
    } else if (pelvis_twist_.has_value()) {
      linear_velocity = {
        pelvis_twist_->twist.linear.x,
        pelvis_twist_->twist.linear.y,
        pelvis_twist_->twist.linear.z,
      };
    } else {
      if (!missing_linear_velocity_warned_) {
        RCLCPP_WARN(
          get_logger(),
          "No pelvis linear velocity received yet. The first 3 observation entries will stay at zero. "
          "Bridge /g1/pelvis/odometry or publish /g1/pelvis_twist.");
        missing_linear_velocity_warned_ = true;
      }
      linear_velocity = {0.0, 0.0, 0.0};
    }

    const std::array<double, 3> angular_velocity = {
      imu_->angular_velocity.x,
      imu_->angular_velocity.y,
      imu_->angular_velocity.z,
    };
    const auto gravity = project_gravity_from_quaternion(
      imu_->orientation.x,
      imu_->orientation.y,
      imu_->orientation.z,
      imu_->orientation.w);

    std::array<float, kObservationDim> observation{};
    std::size_t index = 0;

    const auto append = [&observation, &index](double value) {
      if (index < observation.size()) {
        observation[index++] = static_cast<float>(value);
      }
    };

    // Layout: lin vel, ang vel, projected gravity, qpos_rel, qvel, last_action, command.
    for (const auto value : linear_velocity) {
      append(value);
    }
    for (const auto value : angular_velocity) {
      append(value);
    }
    for (const auto value : gravity) {
      append(value);
    }
    for (std::size_t i = 0; i < kActionDim; ++i) {
      append(ordered->first[i] - kDefaultJointPos[i]);
    }
    for (const auto value : ordered->second) {
      append(value);
    }
    for (const auto value : last_action_) {
      append(value);
    }
    for (const auto value : command_) {
      append(value);
    }

    if (index != observation.size()) {
      RCLCPP_ERROR(
        get_logger(), "Observation has size %zu instead of %zu.", index, observation.size());
      return std::nullopt;
    }

    return observation;
  }

  std::optional<std::array<float, kActionDim>> run_policy(
    const std::array<float, kObservationDim> & observation)
  {
    try {
      // ONNX Runtime expects a batched tensor shaped [1, observation_dim].
      std::array<float, kObservationDim> input_buffer = observation;
      std::array<int64_t, 2> input_shape = {1, static_cast<int64_t>(kObservationDim)};
      auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_buffer.data(),
        input_buffer.size(),
        input_shape.data(),
        input_shape.size());

      const char * input_names[] = {input_name_.c_str()};
      const char * output_names[] = {output_name_.c_str()};
      auto outputs = session_->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names,
        1);

      if (outputs.empty() || !outputs.front().IsTensor()) {
        RCLCPP_ERROR(get_logger(), "ONNX inference returned no tensor output.");
        return std::nullopt;
      }

      auto output_info = outputs.front().GetTensorTypeAndShapeInfo();
      if (output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        RCLCPP_ERROR(get_logger(), "ONNX output is not float32.");
        return std::nullopt;
      }

      if (output_info.GetElementCount() != kActionDim) {
        RCLCPP_ERROR(
          get_logger(), "ONNX output has %zu elements; expected %zu.",
          output_info.GetElementCount(), kActionDim);
        return std::nullopt;
      }

      const float * output_data = outputs.front().GetTensorData<float>();
      std::array<float, kActionDim> action{};
      std::copy(output_data, output_data + kActionDim, action.begin());
      return action;
    } catch (const Ort::Exception & error) {
      RCLCPP_ERROR(get_logger(), "ONNX Runtime error: %s", error.what());
      return std::nullopt;
    }
  }

  void publish_targets(
    const std::array<double, kActionDim> & joint_targets,
    const std::array<float, kActionDim> & raw_action)
  {
    // last_action feeds the next observation; joint targets drive the Gazebo position bridge.
    last_action_publisher_->publish(to_multi_array(raw_action));

    for (std::size_t i = 0; i < kJointOrder.size(); ++i) {
      const auto publisher = joint_publishers_.find(std::string(kJointOrder[i]));
      if (publisher == joint_publishers_.end()) {
        continue;
      }
      Float64 msg;
      msg.data = joint_targets[i];
      publisher->second->publish(msg);
    }
  }

  void run_control_step()
  {
    // Full control step: build obs -> optional debug publish -> policy -> scaled joint targets.
    const auto observation = build_observation();
    if (!observation.has_value()) {
      return;
    }

    if (publish_observation_ && observation_publisher_) {
      observation_publisher_->publish(to_multi_array(*observation));
    }

    if (!activation_sim_time_ns_.has_value()) {
      activation_sim_time_ns_ = current_sim_time_ns_;
    }

    if (
      current_sim_time_ns_.has_value() && activation_sim_time_ns_.has_value() &&
      *current_sim_time_ns_ - *activation_sim_time_ns_ < hold_nominal_pose_duration_ns_)
    {
      // Optional startup phase to let the simulated robot settle before policy control.
      last_action_.fill(0.0F);
      publish_targets(kDefaultJointPos, last_action_);
      return;
    }

    auto raw_action = run_policy(*observation);
    if (!raw_action.has_value()) {
      return;
    }

    if (clip_action_) {
      for (auto & value : *raw_action) {
        value = static_cast<float>(std::clamp(
          static_cast<double>(value), action_clip_min_, action_clip_max_));
      }
    }

    std::array<double, kActionDim> joint_targets{};
    for (std::size_t i = 0; i < kActionDim; ++i) {
      double scaled_action = (*raw_action)[i];
      // These factors are runtime safety knobs, not part of the trained network.
      if (i >= 15) {
        scaled_action *= upper_body_scale_factor_;
      }
      if ((i >= 19 && i <= 21) || (i >= 26 && i <= 28)) {
        scaled_action *= wrist_scale_factor_;
      }
      joint_targets[i] =
        kDefaultJointPos[i] + scaled_action * kActionScale[i] * action_scale_factor_;
    }

    last_action_ = *raw_action;
    publish_targets(joint_targets, last_action_);
  }

  Ort::Env ort_env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;

  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr pelvis_odometry_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr pelvis_twist_sub_;
  rclcpp::Subscription<Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<Clock>::SharedPtr clock_sub_;

  rclcpp::Publisher<Float64MultiArray>::SharedPtr last_action_publisher_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr observation_publisher_;
  std::unordered_map<std::string, rclcpp::Publisher<Float64>::SharedPtr> joint_publishers_;

  std::optional<JointState> joint_state_;
  std::optional<Imu> imu_;
  std::optional<Odometry> pelvis_odometry_;
  std::optional<TwistStamped> pelvis_twist_;
  std::array<double, kCommandDim> command_{0.0, 0.0, 0.0};
  std::array<float, kActionDim> last_action_{};

  bool publish_observation_{false};
  bool clip_action_{false};
  bool missing_linear_velocity_warned_{false};
  double action_clip_min_{-100.0};
  double action_clip_max_{100.0};
  double action_scale_factor_{1.0};
  double upper_body_scale_factor_{1.0};
  double wrist_scale_factor_{1.0};
  int64_t hold_nominal_pose_duration_ns_{0};
  int64_t publish_period_ns_{20'000'000};
  std::optional<int64_t> last_publish_time_ns_;
  std::optional<int64_t> current_sim_time_ns_;
  std::optional<int64_t> activation_sim_time_ns_;
};

}  // namespace policy_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<policy_control::PolicyControllerCpp>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "policy_controller_cpp failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
