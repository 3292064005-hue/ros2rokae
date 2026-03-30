#include "runtime/ros_bindings.hpp"

#include "runtime/ros_service_factory.hpp"

namespace rokae_xmate3_ros2::runtime {

void RosBindings::registerCompatibilityAliases() {
  compatibility_services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetDI>(
      node_, "/xmate3/cobot/get_di", io_program_facade_.get(), &IoProgramFacade::handleGetDI));
  compatibility_services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetDO>(
      node_, "/xmate3/cobot/get_do", io_program_facade_.get(), &IoProgramFacade::handleGetDO));
  compatibility_services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetDI>(
      node_, "/xmate3/cobot/set_di", io_program_facade_.get(), &IoProgramFacade::handleSetDI));
  compatibility_services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetDO>(
      node_, "/xmate3/cobot/set_do", io_program_facade_.get(), &IoProgramFacade::handleSetDO));
  compatibility_services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetAI>(
      node_, "/xmate3/cobot/get_ai", io_program_facade_.get(), &IoProgramFacade::handleGetAI));
  compatibility_services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetAO>(
      node_, "/xmate3/cobot/set_ao", io_program_facade_.get(), &IoProgramFacade::handleSetAO));
  compatibility_services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetSimulationMode>(
      node_, "/xmate3/cobot/set_simulation_mode", control_facade_.get(), &ControlFacade::handleSetSimulationMode));
}

}  // namespace rokae_xmate3_ros2::runtime
