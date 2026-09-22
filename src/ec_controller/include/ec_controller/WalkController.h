#ifndef EC_CONTROLLER_WALK_CONTROLLER_H_
#define EC_CONTROLLER_WALK_CONTROLLER_H_

#include "ec_controller/ControllerBase.h"

#include <ec_radio/msg/radio.hpp>
#include <onnxruntime_cxx_api.h>

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace ec {

// IMU data structure
struct ImuData {
  std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};  // (x, y, z, w)
};

/**
 * @brief Walk controller with reinforcement learning policy
 */
class WalkController : public ControllerBase {
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  void onUpdate(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Helper functions
  ImuData readImu() const;
  std::array<double, 3> computeProjectedGravity(const ImuData &imu) const;
  void buildObservation(const ImuData &imu, float cmd_vx, float cmd_vy, float cmd_wz);
  void updateHistoryBuffer();
  bool runInference();
  void applyActions();

  // Joint parameters
  std::vector<double> default_pos_;
  std::vector<double> walk_kp_;
  std::vector<double> walk_kd_;

  // Action parameters
  int action_size_{23};
  int obs_size_{78};
  int num_history_{10};
  double inference_period_{0.02};
  double inference_accumulator_{0.0};
  std::vector<double> action_scale_;
  double clip_obs_{100.0};
  double clip_actions_{100.0};

  // Inference buffers
  std::vector<float> actions_;
  std::vector<float> obs_;
  std::vector<float> obs_history_;
  int infer_count_{0};
  bool history_filled_{false};

  // ONNX Runtime
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
  Ort::MemoryInfo mem_info_{nullptr};
  std::vector<int64_t> input_shape_;

  // Radio command (lock-free)
  rclcpp::Subscription<ec_radio::msg::Radio>::SharedPtr radio_sub_;
  std::atomic<float> radio_vx_{0.0f};
  std::atomic<float> radio_vy_{0.0f};
  std::atomic<float> radio_wz_{0.0f};
};

} // namespace ec

#endif  // EC_CONTROLLER_WALK_CONTROLLER_H_
