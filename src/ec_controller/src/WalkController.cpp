#include "ec_controller/WalkController.h"

#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>

namespace ec {

// YAML joint order (controllers.yaml): L_leg(6), R_leg(6), waist(3), L_arm(4), R_arm(4)
// NN joint order (from readme.md): L/R interleaved
// kYamlToNn[yaml_idx] = nn_idx
static constexpr int kYamlToNn[23] = {
//  L_hip_p  L_hip_r  L_hip_y  L_knee   L_ank_p  L_ank_r
    0,       3,       6,       9,       13,      17,
//  R_hip_p  R_hip_r  R_hip_y  R_knee   R_ank_p  R_ank_r
    1,       4,       7,       10,      14,      18,
//  waist_y  waist_r  waist_p
    2,       5,       8,
//  L_sho_p  L_sho_r  L_sho_y  L_elbow
    11,      15,      19,      21,
//  R_sho_p  R_sho_r  R_sho_y  R_elbow
    12,      16,      20,      22
};

// IMU interface names (must match ec_imu_hardware exports)
static const std::vector<std::string> kImuInterfaces = {
  "imu_sensor/orientation.x",
  "imu_sensor/orientation.y",
  "imu_sensor/orientation.z",
  "imu_sensor/orientation.w",
  "imu_sensor/angular_velocity.x",
  "imu_sensor/angular_velocity.y",
  "imu_sensor/angular_velocity.z",
  "imu_sensor/linear_acceleration.x",
  "imu_sensor/linear_acceleration.y",
  "imu_sensor/linear_acceleration.z",
};

// Observation indices
static constexpr int kObsImuStart = 0;
static constexpr int kObsImuSize = 6;  // ang_vel(3) + proj_gravity(3)
static constexpr int kObsCmdStart = 6;
static constexpr int kObsCmdSize = 3;  // vx, vy, wz
static constexpr int kObsJointPosStart = 9;
static constexpr int kObsJointVelStart = 32;
static constexpr int kObsPrevActionStart = 55;

ImuData WalkController::readImu() const {
  ImuData imu;
  for (const auto &iface : state_interfaces_) {
    if (iface.get_prefix_name() != "imu_sensor") continue;
    auto val = iface.get_optional();
    if (!val.has_value()) continue;

    const auto &name = iface.get_interface_name();
    if (name == "angular_velocity.x")      imu.angular_velocity[0] = val.value();
    else if (name == "angular_velocity.y") imu.angular_velocity[1] = val.value();
    else if (name == "angular_velocity.z") imu.angular_velocity[2] = val.value();
    else if (name == "orientation.x")      imu.orientation[0] = val.value();
    else if (name == "orientation.y")      imu.orientation[1] = val.value();
    else if (name == "orientation.z")      imu.orientation[2] = val.value();
    else if (name == "orientation.w")      imu.orientation[3] = val.value();
  }
  return imu;
}

std::array<double, 3> WalkController::computeProjectedGravity(const ImuData &imu) const {
  // Compute projected gravity: R^T * [0, 0, -1]
  // q = (x, y, z, w)
  const double qx = imu.orientation[0], qy = imu.orientation[1];
  const double qz = imu.orientation[2], qw = imu.orientation[3];

  return {
    2.0 * (qx * qz - qw * qy) * (-1.0),
    2.0 * (qy * qz + qw * qx) * (-1.0),
    (qw * qw - qx * qx - qy * qy + qz * qz) * (-1.0)
  };
}

void WalkController::buildObservation(const ImuData &imu, float cmd_vx, float cmd_vy, float cmd_wz) {
  const size_t n = jointCount();
  const auto proj_gravity = computeProjectedGravity(imu);

  // IMU data
  obs_[kObsImuStart + 0] = static_cast<float>(imu.angular_velocity[0]);
  obs_[kObsImuStart + 1] = static_cast<float>(imu.angular_velocity[1]);
  obs_[kObsImuStart + 2] = static_cast<float>(imu.angular_velocity[2]);
  obs_[kObsImuStart + 3] = static_cast<float>(proj_gravity[0]);
  obs_[kObsImuStart + 4] = static_cast<float>(proj_gravity[1]);
  obs_[kObsImuStart + 5] = static_cast<float>(proj_gravity[2]);

  // Command
  obs_[kObsCmdStart + 0] = cmd_vx;
  obs_[kObsCmdStart + 1] = cmd_vy;
  obs_[kObsCmdStart + 2] = cmd_wz;

  // Joint positions (remapped to NN order)
  for (size_t i = 0; i < n; ++i) {
    obs_[kObsJointPosStart + kYamlToNn[i]] =
      static_cast<float>(jointState(i).position - default_pos_[i]);
  }

  // Joint velocities (remapped to NN order)
  for (size_t i = 0; i < n; ++i) {
    obs_[kObsJointVelStart + kYamlToNn[i]] =
      static_cast<float>(jointState(i).velocity);
  }

  // Previous actions
  for (int i = 0; i < action_size_; ++i) {
    obs_[kObsPrevActionStart + i] = actions_[i];
  }

  // Clip observations
  const float clip = static_cast<float>(clip_obs_);
  for (auto &v : obs_) {
    v = std::clamp(v, -clip, clip);
  }
}

void WalkController::updateHistoryBuffer() {
  if (!history_filled_) {
    // First frame: fill entire buffer with current observation
    for (int h = 0; h < num_history_; ++h) {
      std::memcpy(obs_history_.data() + h * obs_size_,
                   obs_.data(), obs_size_ * sizeof(float));
    }
    history_filled_ = true;
  } else {
    // Normal: shift left and append latest
    std::memmove(obs_history_.data(),
                 obs_history_.data() + obs_size_,
                 (num_history_ - 1) * obs_size_ * sizeof(float));
    std::memcpy(obs_history_.data() + (num_history_ - 1) * obs_size_,
                 obs_.data(), obs_size_ * sizeof(float));
  }
}

bool WalkController::runInference() {
  if (!ort_session_) return false;

  try {
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info_, obs_history_.data(), obs_history_.size(),
      input_shape_.data(), input_shape_.size());

    auto output_tensors = ort_session_->Run(
      Ort::RunOptions{nullptr},
      input_names_.data(), &input_tensor, 1,
      output_names_.data(), 1);

    const float *raw_actions = output_tensors[0].GetTensorData<float>();
    const float clip = static_cast<float>(clip_actions_);

    for (int i = 0; i < action_size_; ++i) {
      actions_[i] = std::clamp(raw_actions[i], -clip, clip);
    }

    infer_count_++;
    return true;
  } catch (const Ort::Exception &e) {
    RCLCPP_ERROR_THROTTLE(logger(), *get_node()->get_clock(), 1000,
      "ONNX inference failed: %s", e.what());
    return false;
  }
}

void WalkController::applyActions() {
  const size_t n = jointCount();

  for (size_t i = 0; i < n; ++i) {
    auto &cmd = jointCommand(i);
    const int nn_idx = kYamlToNn[i];
    const float scaled_action = (nn_idx < action_size_)
      ? static_cast<float>(action_scale_[i]) * actions_[nn_idx]
      : 0.0f;

    cmd.position = default_pos_[i] + scaled_action;
    cmd.velocity = 0.0;
    cmd.effort = 0.0;
    cmd.kp = walk_kp_[i];
    cmd.kd = walk_kd_[i];
    cmd.torque_mode = kCoupledJoint[i] ? 2.0 : 0.0;
  }
}

controller_interface::CallbackReturn WalkController::on_configure(
  const rclcpp_lifecycle::State &previous_state)
{
  auto ret = ControllerBase::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) return ret;

  auto node = get_node();
  const size_t n = jointCount();

  // Joint parameters
  node->declare_parameter<std::vector<double>>("walk_pos", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("walk_kp", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("walk_kd", std::vector<double>{});

  default_pos_ = node->get_parameter("walk_pos").as_double_array();
  walk_kp_ = node->get_parameter("walk_kp").as_double_array();
  walk_kd_ = node->get_parameter("walk_kd").as_double_array();

  // Validate joint parameter sizes
  if (default_pos_.size() < n || walk_kp_.size() < n || walk_kd_.size() < n) {
    RCLCPP_ERROR(logger(), "Parameter array size mismatch: expected %zu, got pos=%zu kp=%zu kd=%zu",
                 n, default_pos_.size(), walk_kp_.size(), walk_kd_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Action parameters
  node->declare_parameter<int>("action.action_size", 23);
  node->declare_parameter<int>("action.observation_size", 78);
  node->declare_parameter<int>("action.num_history", 10);
  node->declare_parameter<int>("action.policy_freq", 50);
  node->declare_parameter<std::vector<double>>("action.action_scale", std::vector<double>(23, 0.25));

  action_size_ = node->get_parameter("action.action_size").as_int();
  obs_size_ = node->get_parameter("action.observation_size").as_int();
  num_history_ = node->get_parameter("action.num_history").as_int();
  int policy_freq = node->get_parameter("action.policy_freq").as_int();
  action_scale_ = node->get_parameter("action.action_scale").as_double_array();

  inference_period_ = 1.0 / policy_freq;

  // Validate observation size: 9 (ang_vel + gravity + cmd) + 2*n (pos + vel) + action_size
  const int expected_obs = kObsImuSize + kObsCmdSize + 2 * static_cast<int>(n) + action_size_;
  if (obs_size_ != expected_obs) {
    RCLCPP_ERROR(logger(), "observation_size mismatch: expected %d, got %d",
                 expected_obs, obs_size_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (static_cast<int>(action_scale_.size()) < action_size_) {
    RCLCPP_ERROR(logger(), "action_scale size %zu < action_size %d",
                 action_scale_.size(), action_size_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // Normalization
  node->declare_parameter<double>("normalization.clip_scales.clip_observations", 100.0);
  node->declare_parameter<double>("normalization.clip_scales.clip_actions", 100.0);
  clip_obs_ = node->get_parameter("normalization.clip_scales.clip_observations").as_double();
  clip_actions_ = node->get_parameter("normalization.clip_scales.clip_actions").as_double();

  // Policy path
  node->declare_parameter<std::string>("policy_path", "");
  std::string policy_path = node->get_parameter("policy_path").as_string();

  // Resolve relative path against package share directory
  if (!policy_path.empty() && !std::filesystem::path(policy_path).is_absolute()) {
    std::string share_dir = ament_index_cpp::get_package_share_directory("ec_controller");
    policy_path = share_dir + "/" + policy_path;
  }

  // Initialize buffers
  actions_.assign(action_size_, 0.0f);
  obs_.resize(obs_size_);
  obs_history_.assign(num_history_ * obs_size_, 0.0f);

  // Load ONNX model
  if (!policy_path.empty()) {
    try {
      ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "WalkController");
      Ort::SessionOptions session_opts;
      session_opts.SetIntraOpNumThreads(1);
      ort_session_ = std::make_unique<Ort::Session>(*ort_env_, policy_path.c_str(), session_opts);

      Ort::AllocatorWithDefaultOptions alloc;
      input_name_ = ort_session_->GetInputNameAllocated(0, alloc).get();
      output_name_ = ort_session_->GetOutputNameAllocated(0, alloc).get();
      input_names_ = {input_name_.c_str()};
      output_names_ = {output_name_.c_str()};

      mem_info_ = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
      input_shape_ = {1, num_history_ * obs_size_};

      RCLCPP_INFO(logger(), "Loaded policy: %s (obs=%d, act=%d, history=%d)",
                  policy_path.c_str(), obs_size_, action_size_, num_history_);
    } catch (const Ort::Exception &e) {
      RCLCPP_ERROR(logger(), "Failed to load ONNX model: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_WARN(logger(), "No policy_path set, inference disabled");
  }

  // Subscribe to radio for command input
  radio_sub_ = node->create_subscription<ec_radio::msg::Radio>(
    "/radio", 10,
    [this](const ec_radio::msg::Radio::SharedPtr msg) {
      radio_vx_.store(msg->vx);
      radio_vy_.store(msg->vy);
      radio_wz_.store(msg->wz);
    });

  RCLCPP_INFO(logger(), "WalkController configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WalkController::on_activate(
  const rclcpp_lifecycle::State &previous_state)
{
  auto ret = ControllerBase::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) return ret;

  // Initialize observation from current state
  const auto imu = readImu();
  buildObservation(imu, 0.0f, 0.0f, 0.0f);

  // Fill history buffer with current observation
  for (int h = 0; h < num_history_; ++h) {
    std::memcpy(obs_history_.data() + h * obs_size_,
                 obs_.data(), obs_size_ * sizeof(float));
  }
  history_filled_ = true;
  actions_.assign(action_size_, 0.0f);

  RCLCPP_INFO(logger(), "WalkController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WalkController::on_deactivate(
  const rclcpp_lifecycle::State &previous_state)
{
  history_filled_ = false;
  actions_.assign(action_size_, 0.0f);
  return ControllerBase::on_deactivate(previous_state);
}

controller_interface::InterfaceConfiguration WalkController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Joint state interfaces
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
    config.names.push_back(joint_name + "/effort");
  }

  // IMU state interfaces
  for (const auto &iface : kImuInterfaces) {
    config.names.push_back(iface);
  }

  return config;
}

void WalkController::onUpdate(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
  // Read IMU and radio
  const auto imu = readImu();
  const float cmd_vx = radio_vx_.load();
  const float cmd_vy = radio_vy_.load();
  const float cmd_wz = radio_wz_.load();

  // Accumulate time for inference
  inference_accumulator_ += period.seconds();

  if (inference_accumulator_ >= inference_period_ && ort_session_) {
    inference_accumulator_ -= inference_period_;

    // Build observation and run inference
    buildObservation(imu, cmd_vx, cmd_vy, cmd_wz);
    updateHistoryBuffer();
    runInference();
  }

  // Apply actions to joints
  applyActions();
}

} // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::WalkController, controller_interface::ControllerInterface)
