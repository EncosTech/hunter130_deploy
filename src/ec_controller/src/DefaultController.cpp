#include <pluginlib/class_list_macros.hpp>
#include "ec_controller/PositionController.h"

namespace ec {

class DefaultController : public PositionController {
protected:
  std::string paramPrefix() const override { return "default"; }
};

} // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::DefaultController, controller_interface::ControllerInterface)
