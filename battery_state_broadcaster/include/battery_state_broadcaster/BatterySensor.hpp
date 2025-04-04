
#include <limits>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/detail/battery_state__struct.hpp>

//
// inspired by https://github.com/HarvestX/h6x_ros2_controllers/blob/humble/battery_state_broadcaster/include/semantic_components/battery_state.hpp
//

namespace battery_state_broadcaster
{
class BatterySensor : public semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatterySensor(const std::string& name, const std::vector<std::string> & interfaces, rclcpp::Logger logger)
    : semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>(name, interfaces.size())
  {
    for (const auto & interface : interfaces) {
      std::string interface_name = name_ + "/" + interface;
      interface_names_.emplace_back(interface_name);
      RCLCPP_INFO(logger, "Interface '%s' configured", interface_name.c_str());
    }
  }

  virtual ~BatterySensor() = default;

  double get_voltage()
  {
    const auto val = state_interfaces_[0].get().get_optional();
    voltage_ = val.has_value() ? val.value() : NAN;
    return voltage_;
  }

  void populate_message_from_interfaces(sensor_msgs::msg::BatteryState& message)
  {
    // seed the message with dynamic values - only from existing interfaces:
    for (const auto & state_interface : state_interfaces_) {
      const auto & name = state_interface.get().get_interface_name();
      const auto val = state_interface.get().get_optional();
      if (name == "voltage") {
        message.voltage = val.has_value() ? val.value() : NAN;
      } else if (name == "temperature") {
        message.temperature = val.has_value() ? val.value() : NAN;
      } else if (name == "charge") {
        message.charge = val.has_value() ? val.value() : NAN;
      } else if (name == "current") {
        message.current = val.has_value() ? val.value() : NAN;
      } else if (name == "capacity") {
        message.capacity = val.has_value() ? val.value() : NAN;
      } else if (name == "percentage") {
        message.percentage = val.has_value() ? val.value() : NAN;
      } else if (name == "power_supply_health") {
        message.power_supply_health = static_cast<uint8_t>(round(val.has_value() ? val.value() : 0.0));
      } else if (name == "power_supply_status") {
        message.power_supply_status = static_cast<uint8_t>(round(val.has_value() ? val.value() : 0.0));
      } else if (name == "present") {
        message.present = static_cast<int>(round(val.has_value() ? val.value() : 0.0)) == 0 ? false : true;
      }
    }
  }

private:
  double voltage_ = 0.0;
};
}  // namespace battery_state_broadcaster
