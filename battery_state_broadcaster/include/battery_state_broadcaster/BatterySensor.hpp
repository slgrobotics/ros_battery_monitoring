
#include <limits>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/detail/battery_state__struct.hpp>

//
// inspired by
// https://github.com/HarvestX/h6x_ros2_controllers/blob/humble/battery_state_broadcaster/include/semantic_components/battery_state.hpp
//

namespace battery_state_broadcaster
{
class BatterySensor : public semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatterySensor(
    const std::string& name, const std::vector<std::string>& interfaces)
    : semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>(name, interfaces.size())
  {
    for (const auto& interface : interfaces)
    {
      std::string interface_name = name_ + "/" + interface;
      interface_names_.emplace_back(interface_name);
    }
  }

  virtual ~BatterySensor() = default;

  void populate_message_from_interfaces(sensor_msgs::msg::BatteryState& message)
  {
    // do this only for the existing interfaces
    for (const auto& state_interface : state_interfaces_)
    {
      const auto& name = state_interface.get().get_interface_name();
      if (const auto& data = state_interface.get().get_optional(1); data != std::nullopt) {
          if (name == "voltage")
          {
            message.voltage = data.value();
          }
          else if (name == "temperature")
          {
            message.temperature = data.value();
          }
          else if (name == "charge")
          {
            message.charge = data.value();
          }
          else if (name == "current")
          {
            message.current = data.value();
          }
          else if (name == "capacity")
          {
            message.capacity = data.value();
          }
          else if (name == "percentage")
          {
            message.percentage = data.value();
          }
          else if (name == "power_supply_health")
          {
            message.power_supply_health =
                static_cast<uint8_t>(std::round(data.value()));
          }
          else if (name == "power_supply_status")
          {
            message.power_supply_status =
                static_cast<uint8_t>(std::round(data.value()));
          }
          else if (name == "present")
          {
            message.present =
                static_cast<int>(
                    std::round(data.value())) == 0 ? false : true;
          }
      }
    }
  }

private:
  double voltage_ = 0.0;
};
}  // namespace battery_state_broadcaster
