#include "battery_state_broadcaster/BatteryStateBroadcaster.hpp"
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

//
// Some code from https://github.com/HarvestX/h6x_ros2_controllers/blob/humble/battery_state_broadcaster/src/battery_state_broadcaster.cpp
//

namespace battery_state_broadcaster
{
controller_interface::CallbackReturn BatteryStateBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BatteryStateBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::string sensor_name = get_node()->get_parameter("sensor_name").as_string();

  params_ = param_listener_->get_params();

  // Old way of constructing BatterySensor in Jazzy:
  //battery_sensor_ = std::make_unique<BatterySensor>(BatterySensor(sensor_name, params_.state_interfaces, get_node()->get_logger()));

  // In Kilted some controller plugin types (like BatterySensor) are now non-copyable / non-movable. Works in Jazzy too:
  battery_sensor_ = std::make_unique<BatterySensor>(sensor_name, params_.state_interfaces, get_node()->get_logger());

  try {
    // register ft sensor data publisher
    battery_state_pub_ =
        get_node()->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(battery_state_pub_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  //
  // A better Kilted way of filling message:
  //
  // realtime_publisher_->try_publish([&](auto &msg) {
  //   msg.design_capacity = static_cast<float>(params_.design_capacity);
  //   ...
  // });
  sensor_msgs::msg::BatteryState msg;

  // A Jazzy and Kilted compatible way of filling message in the interim:
  // seed the message with static values from parameters and defaults for dynamic members:
  msg.header.frame_id = params_.frame_id;
  msg.design_capacity = static_cast<float>(params_.design_capacity);
  msg.voltage = std::numeric_limits<float>::quiet_NaN();
  msg.temperature = std::numeric_limits<float>::quiet_NaN();
  msg.charge = std::numeric_limits<float>::quiet_NaN();
  msg.current = std::numeric_limits<float>::quiet_NaN();
  msg.capacity = std::numeric_limits<float>::quiet_NaN();
  msg.percentage = std::numeric_limits<float>::quiet_NaN();
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = params_.power_supply_technology;
  msg.present = false;
  msg.location = params_.location;
  msg.serial_number = params_.serial_number;

  if(!realtime_publisher_->try_publish(msg)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to lock realtime publisher at configure stage \n");
    return CallbackReturn::ERROR;
  } 

  RCLCPP_DEBUG(get_node()->get_logger(), "on_configure() successful");

  return CallbackReturn::SUCCESS;
}

[[nodiscard]] controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

[[nodiscard]] controller_interface::InterfaceConfiguration BatteryStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = battery_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn
BatteryStateBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  battery_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BatteryStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  battery_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BatteryStateBroadcaster::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& /*period*/)
{
  sensor_msgs::msg::BatteryState msg;

  if (realtime_publisher_) {
    msg.header.stamp = time;
    battery_sensor_->populate_message_from_interfaces(msg);
    realtime_publisher_->try_publish(msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace battery_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(battery_state_broadcaster::BatteryStateBroadcaster, controller_interface::ControllerInterface)
