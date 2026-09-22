#ifndef EC_CONTROLLER_POSITION_CONTROLLER_H_
#define EC_CONTROLLER_POSITION_CONTROLLER_H_

#include "ec_controller/ControllerBase.h"

#include <string>
#include <vector>

namespace ec {

/**
 * @brief Base class for position-hold controllers (Default, Stand)
 *
 * Loads position/kp/kd parameters and applies them each update cycle.
 */
class PositionController : public ControllerBase {
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &previous_state) override;

protected:
  void onUpdate(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // Parameter prefix (e.g., "default" or "stand")
  virtual std::string paramPrefix() const = 0;

  std::vector<double> target_pos_;
  std::vector<double> kp_;
  std::vector<double> kd_;
};

} // namespace ec

#endif  // EC_CONTROLLER_POSITION_CONTROLLER_H_
