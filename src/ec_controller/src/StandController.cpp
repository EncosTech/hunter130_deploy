#include <pluginlib/class_list_macros.hpp>
#include "ec_controller/PositionController.h"

namespace ec {

class StandController : public PositionController {
protected:
  std::string paramPrefix() const override { return "stand"; }
};

} // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::StandController, controller_interface::ControllerInterface)
