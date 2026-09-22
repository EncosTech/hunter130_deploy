#include "ec_controller/PositionController.h"

namespace ec {

controller_interface::CallbackReturn PositionController::on_configure(
  const rclcpp_lifecycle::State &previous_state)
{
  auto ret = ControllerBase::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) return ret;

  auto node = get_node();
  const std::string prefix = paramPrefix();

  // Declare and load parameters
  node->declare_parameter<std::vector<double>>(prefix + "_pos", std::vector<double>{});
  node->declare_parameter<std::vector<double>>(prefix + "_kp", std::vector<double>{});
  node->declare_parameter<std::vector<double>>(prefix + "_kd", std::vector<double>{});

  target_pos_ = node->get_parameter(prefix + "_pos").as_double_array();
  kp_ = node->get_parameter(prefix + "_kp").as_double_array();
  kd_ = node->get_parameter(prefix + "_kd").as_double_array();

  // Validate parameter sizes
  const size_t n = jointCount();
  if (target_pos_.size() < n || kp_.size() < n || kd_.size() < n) {
    RCLCPP_ERROR(logger(), "Parameter array size mismatch: expected %zu, got pos=%zu kp=%zu kd=%zu",
                 n, target_pos_.size(), kp_.size(), kd_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void PositionController::onUpdate(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const size_t n = jointCount();
  for (size_t i = 0; i < n; ++i) {
    auto& cmd = jointCommand(i);
    cmd.position = target_pos_[i];
    cmd.velocity = 0.0;
    cmd.effort = 0.0;
    cmd.kp = kp_[i];
    cmd.kd = kd_[i];
    cmd.torque_mode = kCoupledJoint[i] ? 2.0 : 0.0;
  }
}

} // namespace ec
