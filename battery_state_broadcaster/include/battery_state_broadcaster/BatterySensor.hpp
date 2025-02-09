
#include <limits>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/detail/battery_state__struct.hpp>

namespace battery_state_broadcaster
{
class BatterySensor : public semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatterySensor(const std::string& name, const std::vector<std::string> & interfaces, rclcpp::Logger logger)
    : semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>(name, interfaces.size())
  {
    for (const auto & interface : interfaces) {
      std::string interface_name = this->name_ + "/" + interface;
      this->interface_names_.emplace_back(interface_name);
      RCLCPP_INFO(logger, "Interface '%s' configured", interface_name.c_str());
    }
  }

  virtual ~BatterySensor() = default;

  double get_voltage()
  {
    voltage_ = state_interfaces_[0].get().get_value();
    return voltage_;
  }

  bool get_values_as_message(sensor_msgs::msg::BatteryState& message)
  {
    //get_voltage();
    //message.voltage = static_cast<float>(voltage_);
    //return true;

    for (const auto & state_interface : this->state_interfaces_) {
      const auto & name = state_interface.get().get_interface_name();
      const auto & value = state_interface.get().get_value();
      if (name == "voltage") {
        message.voltage = value;
      } else if (name == "temperature") {
        message.temperature = value;
      } else if (name == "charge") {
        message.charge = value;
      } else if (name == "current") {
        message.current = value;
      } else if (name == "capacity") {
        message.capacity = value;
      } else if (name == "percentage") {
        message.percentage = value;
      } else if (name == "power_supply_health") {
        message.power_supply_health = static_cast<uint8_t>(value);
      } else if (name == "power_supply_status") {
        message.power_supply_status = static_cast<uint8_t>(value);
      }
    }
    return false;
  }

private:
  double voltage_ = 0.0;
};
}  // namespace battery_state_broadcaster
