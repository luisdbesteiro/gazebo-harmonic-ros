#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "geometry_msgs/msg/twist.hpp"
#include "policy_control/policy_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

constexpr char kDefaultOnnxWorkspacePath[] = "/workspace/results/2026-01-26_10-42-02.onnx";

std::string sdf_string(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  const std::string & default_value)
{
  return sdf->HasElement(name) ? sdf->Get<std::string>(name) : default_value;
}

double sdf_double(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  double default_value)
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : default_value;
}

bool sdf_bool(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  bool default_value)
{
  return sdf->HasElement(name) ? sdf->Get<bool>(name) : default_value;
}

std::array<double, 3> world_to_body(
  const gz::math::Quaterniond & orientation,
  const gz::math::Vector3d & vector)
{
  const auto local = orientation.Inverse().RotateVector(vector);
  return {local.X(), local.Y(), local.Z()};
}

std::array<double, 3> project_gravity(const gz::math::Quaterniond & orientation)
{
  return world_to_body(orientation, gz::math::Vector3d(0.0, 0.0, -1.0));
}

}  // namespace

namespace policy_control
{

using SimDuration = std::chrono::steady_clock::duration;

class PolicyGzSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  PolicyGzSystem()
  : ort_env_(ORT_LOGGING_LEVEL_WARNING, "policy_gz_system")
  {
  }

  ~PolicyGzSystem() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (ros_thread_.joinable()) {
      ros_thread_.join();
    }
    if (ros_context_ && ros_context_->is_valid()) {
      ros_context_->shutdown("policy_gz_system shutdown");
    }
  }

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      throw std::runtime_error("policy_gz_system must be attached to a model entity.");
    }

    onnx_model_path_ = sdf_string(sdf, "onnx_model_path", kDefaultOnnxWorkspacePath);
    cmd_vel_topic_ = sdf_string(sdf, "cmd_vel_topic", "/cmd_vel");
    cmd_pos_prefix_ = strip_trailing_slash(sdf_string(sdf, "cmd_pos_prefix", "/g1/cmd_pos"));
    metrics_topic_ = sdf_string(sdf, "metrics_topic", "/g1/policy_metrics");
    pelvis_link_name_ = sdf_string(sdf, "pelvis_link_name", "pelvis");
    control_rate_hz_ = sdf_double(sdf, "control_rate_hz", 50.0);
    publish_metrics_ = sdf_bool(sdf, "publish_metrics", false);
    clip_action_ = sdf_bool(sdf, "clip_action", false);
    action_clip_min_ = sdf_double(sdf, "action_clip_min", -100.0);
    action_clip_max_ = sdf_double(sdf, "action_clip_max", 100.0);
    action_scale_factor_ = sdf_double(sdf, "action_scale_factor", 1.0);
    upper_body_scale_factor_ = sdf_double(sdf, "upper_body_scale_factor", 1.0);
    wrist_scale_factor_ = sdf_double(sdf, "wrist_scale_factor", 1.0);
    hold_nominal_pose_duration_s_ = sdf_double(sdf, "hold_nominal_pose_duration_s", 0.0);
    control_period_ = std::chrono::duration_cast<SimDuration>(
      control_rate_hz_ > 0.0 ?
      std::chrono::duration<double>(1.0 / control_rate_hz_) :
      std::chrono::duration<double>(0.02));

    load_session();
    configure_entities(ecm);
    configure_transport();
    configure_ros();

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "policy_gz_system ready on model %s. ONNX: %s, cmd_vel: %s, cmd_pos prefix: %s",
      model_.Name(ecm).c_str(),
      onnx_model_path_.c_str(),
      cmd_vel_topic_.c_str(),
      cmd_pos_prefix_.c_str());
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }

    if (!last_control_time_.has_value()) {
      last_control_time_ = info.simTime - control_period_;
    }

    if (info.simTime - *last_control_time_ < control_period_) {
      return;
    }

    last_control_time_ = info.simTime;
    if (!activation_time_.has_value()) {
      activation_time_ = info.simTime;
    }

    run_control_step(info, ecm);
  }

private:
  void load_session()
  {
    if (onnx_model_path_.empty()) {
      throw std::runtime_error("No ONNX model path configured for policy_gz_system.");
    }
    if (!std::filesystem::exists(onnx_model_path_)) {
      throw std::runtime_error("ONNX model not found: " + onnx_model_path_);
    }

    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = std::make_unique<Ort::Session>(ort_env_, onnx_model_path_.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = session_->GetInputNameAllocated(0, allocator);
    auto output_name = session_->GetOutputNameAllocated(0, allocator);
    input_name_ = input_name.get();
    output_name_ = output_name.get();
  }

  void configure_entities(gz::sim::EntityComponentManager & ecm)
  {
    joints_.clear();
    joints_.reserve(kJointOrder.size());

    for (const auto joint_name : kJointOrder) {
      const auto joint_entity = model_.JointByName(ecm, std::string(joint_name));
      if (joint_entity == gz::sim::kNullEntity) {
        throw std::runtime_error("Missing joint for policy_gz_system: " + std::string(joint_name));
      }

      gz::sim::Joint joint(joint_entity);
      joint.EnablePositionCheck(ecm);
      joint.EnableVelocityCheck(ecm);
      joints_.push_back(joint);
    }

    const auto pelvis_entity = model_.LinkByName(ecm, pelvis_link_name_);
    if (pelvis_entity == gz::sim::kNullEntity) {
      throw std::runtime_error("Missing pelvis link for policy_gz_system: " + pelvis_link_name_);
    }

    pelvis_link_ = gz::sim::Link(pelvis_entity);
    pelvis_link_.EnableVelocityChecks(ecm);
  }

  void configure_transport()
  {
    cmd_pos_publishers_.clear();
    cmd_pos_publishers_.reserve(kJointOrder.size());

    for (const auto joint_name : kJointOrder) {
      cmd_pos_publishers_.push_back(
        gz_node_.Advertise<gz::msgs::Double>(
          cmd_pos_prefix_ + "/" + std::string(joint_name)));
    }
  }

  void configure_ros()
  {
    ros_context_ = std::make_shared<rclcpp::Context>();
    ros_context_->init(0, nullptr);

    rclcpp::NodeOptions options;
    options.context(ros_context_);
    ros_node_ = std::make_shared<rclcpp::Node>("policy_gz_system", options);

    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = {msg->linear.x, msg->linear.y, msg->angular.z};
      });

    if (publish_metrics_) {
      metrics_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        metrics_topic_, 10);
    }

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = ros_context_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
      executor_options);
    executor_->add_node(ros_node_);
    ros_thread_ = std::thread([this]() { executor_->spin(); });
  }

  std::optional<std::array<float, kObservationDim>> build_observation(
    const gz::sim::EntityComponentManager & ecm)
  {
    const auto pelvis_pose = pelvis_link_.WorldPose(ecm);
    const auto pelvis_linear_velocity = pelvis_link_.WorldLinearVelocity(ecm);
    const auto pelvis_angular_velocity = pelvis_link_.WorldAngularVelocity(ecm);
    if (!pelvis_pose || !pelvis_linear_velocity || !pelvis_angular_velocity) {
      warn_throttled("Waiting for pelvis pose/velocity components in policy_gz_system.");
      return std::nullopt;
    }

    std::array<float, kObservationDim> observation{};
    std::size_t index = 0;

    const auto append = [&observation, &index](double value) {
      if (index < observation.size()) {
        observation[index++] = static_cast<float>(value);
      }
    };

    const auto linear_velocity = world_to_body(pelvis_pose->Rot(), *pelvis_linear_velocity);
    const auto angular_velocity = world_to_body(pelvis_pose->Rot(), *pelvis_angular_velocity);
    const auto gravity = project_gravity(pelvis_pose->Rot());

    for (const auto value : linear_velocity) {
      append(value);
    }
    for (const auto value : angular_velocity) {
      append(value);
    }
    for (const auto value : gravity) {
      append(value);
    }

    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto position = joints_[i].Position(ecm);
      if (!position || position->empty()) {
        warn_throttled("Waiting for joint position components in policy_gz_system.");
        return std::nullopt;
      }
      append((*position)[0] - kDefaultJointPos[i]);
    }

    for (const auto & joint : joints_) {
      const auto velocity = joint.Velocity(ecm);
      append((velocity && !velocity->empty()) ? (*velocity)[0] : 0.0);
    }

    for (const auto value : last_action_) {
      append(value);
    }

    const auto command = latest_command();
    for (const auto value : command) {
      append(value);
    }

    if (index != observation.size()) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "policy_gz_system observation has size %zu instead of %zu.",
        index,
        observation.size());
      return std::nullopt;
    }

    return observation;
  }

  std::optional<std::array<float, kActionDim>> run_policy(
    const std::array<float, kObservationDim> & observation)
  {
    try {
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
        RCLCPP_ERROR(ros_node_->get_logger(), "ONNX inference returned no tensor output.");
        return std::nullopt;
      }

      auto output_info = outputs.front().GetTensorTypeAndShapeInfo();
      if (output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        RCLCPP_ERROR(ros_node_->get_logger(), "ONNX output is not float32.");
        return std::nullopt;
      }

      if (output_info.GetElementCount() != kActionDim) {
        RCLCPP_ERROR(
          ros_node_->get_logger(),
          "ONNX output has %zu elements; expected %zu.",
          output_info.GetElementCount(),
          kActionDim);
        return std::nullopt;
      }

      const float * output_data = outputs.front().GetTensorData<float>();
      std::array<float, kActionDim> action{};
      std::copy(output_data, output_data + kActionDim, action.begin());
      return action;
    } catch (const Ort::Exception & error) {
      RCLCPP_ERROR(ros_node_->get_logger(), "ONNX Runtime error: %s", error.what());
      return std::nullopt;
    }
  }

  void run_control_step(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm)
  {
    const auto start = std::chrono::steady_clock::now();
    const auto observation = build_observation(ecm);
    const auto observation_done = std::chrono::steady_clock::now();
    if (!observation) {
      return;
    }

    std::array<float, kActionDim> raw_action{};
    if (
      activation_time_.has_value() &&
      std::chrono::duration<double>(info.simTime - *activation_time_).count() <
      hold_nominal_pose_duration_s_)
    {
      raw_action.fill(0.0F);
      publish_targets(kDefaultJointPos, raw_action);
      last_action_ = raw_action;
      return;
    }

    auto policy_action = run_policy(*observation);
    const auto inference_done = std::chrono::steady_clock::now();
    if (!policy_action) {
      return;
    }

    raw_action = *policy_action;
    if (clip_action_) {
      for (auto & value : raw_action) {
        value = static_cast<float>(std::clamp(
          static_cast<double>(value), action_clip_min_, action_clip_max_));
      }
    }

    std::array<double, kActionDim> joint_targets{};
    for (std::size_t i = 0; i < kActionDim; ++i) {
      double scaled_action = raw_action[i];
      if (i >= 15) {
        scaled_action *= upper_body_scale_factor_;
      }
      if ((i >= 19 && i <= 21) || (i >= 26 && i <= 28)) {
        scaled_action *= wrist_scale_factor_;
      }
      joint_targets[i] =
        kDefaultJointPos[i] + scaled_action * kActionScale[i] * action_scale_factor_;
    }

    publish_targets(joint_targets, raw_action);
    last_action_ = raw_action;

    const auto publish_done = std::chrono::steady_clock::now();
    publish_metrics(start, observation_done, inference_done, publish_done);
  }

  void publish_targets(
    const std::array<double, kActionDim> & joint_targets,
    const std::array<float, kActionDim> &)
  {
    for (std::size_t i = 0; i < joint_targets.size(); ++i) {
      gz::msgs::Double msg;
      msg.set_data(joint_targets[i]);
      cmd_pos_publishers_[i].Publish(msg);
    }
  }

  void publish_metrics(
    const std::chrono::steady_clock::time_point & start,
    const std::chrono::steady_clock::time_point & observation_done,
    const std::chrono::steady_clock::time_point & inference_done,
    const std::chrono::steady_clock::time_point & publish_done)
  {
    if (!metrics_pub_) {
      return;
    }

    const auto ms = [](const auto & a, const auto & b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      ms(start, observation_done),
      ms(observation_done, inference_done),
      ms(inference_done, publish_done),
      ms(start, publish_done),
    };
    metrics_pub_->publish(msg);
  }

  std::array<double, kCommandDim> latest_command() const
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
  }

  static std::string strip_trailing_slash(std::string value)
  {
    while (value.size() > 1 && value.back() == '/') {
      value.pop_back();
    }
    return value;
  }

  void warn_throttled(const std::string & message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (
      !last_warning_time_.has_value() ||
      now - *last_warning_time_ > std::chrono::seconds(5))
    {
      last_warning_time_ = now;
      RCLCPP_WARN(ros_node_->get_logger(), "%s", message.c_str());
    }
  }

  gz::sim::Model model_;
  gz::sim::Link pelvis_link_;
  std::vector<gz::sim::Joint> joints_;

  gz::transport::Node gz_node_;
  std::vector<gz::transport::Node::Publisher> cmd_pos_publishers_;

  rclcpp::Context::SharedPtr ros_context_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
  std::thread ros_thread_;

  Ort::Env ort_env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;

  mutable std::mutex command_mutex_;
  std::array<double, kCommandDim> command_{0.0, 0.0, 0.0};
  std::array<float, kActionDim> last_action_{};

  std::string onnx_model_path_{kDefaultOnnxWorkspacePath};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string cmd_pos_prefix_{"/g1/cmd_pos"};
  std::string metrics_topic_{"/g1/policy_metrics"};
  std::string pelvis_link_name_{"pelvis"};
  double control_rate_hz_{50.0};
  double action_clip_min_{-100.0};
  double action_clip_max_{100.0};
  double action_scale_factor_{1.0};
  double upper_body_scale_factor_{1.0};
  double wrist_scale_factor_{1.0};
  double hold_nominal_pose_duration_s_{0.0};
  bool publish_metrics_{false};
  bool clip_action_{false};
  SimDuration control_period_{std::chrono::duration_cast<SimDuration>(
    std::chrono::duration<double>(0.02))};
  std::optional<std::chrono::steady_clock::time_point> last_warning_time_;
  std::optional<SimDuration> last_control_time_;
  std::optional<SimDuration> activation_time_;
};

}  // namespace policy_control

GZ_ADD_PLUGIN(
  policy_control::PolicyGzSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(policy_control::PolicyGzSystem, "policy_control::PolicyGzSystem")
