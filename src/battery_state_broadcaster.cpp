// Copyright (c) 2025, b-robotized
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "battery_state_broadcaster/battery_state_broadcaster.hpp"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace battery_state_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
const size_t MAX_LENGTH = 64;

BatteryStateBroadcaster::BatteryStateBroadcaster() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<battery_state_broadcaster::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  state_joints_ = params_.state_joints;
  battery_presence_.resize(state_joints_.size(), false);

  try
  {
    battery_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
      "~/battery_state", rclcpp::SystemDefaultsQoS());

    battery_state_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>(
        battery_state_publisher_);

    raw_battery_states_publisher_ = get_node()->create_publisher<control_msgs::msg::BatteryStates>(
      "~/raw_battery_states", rclcpp::SystemDefaultsQoS());

    raw_battery_states_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::BatteryStates>>(
        raw_battery_states_publisher_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory in state publisher depending on the message type
  battery_state_realtime_publisher_->lock();
  battery_state_realtime_publisher_->msg_.location.reserve(MAX_LENGTH);
  battery_state_realtime_publisher_->msg_.serial_number.reserve(MAX_LENGTH);
  battery_state_realtime_publisher_->unlock();

  raw_battery_states_realtime_publisher_->lock();
  auto & msg = raw_battery_states_realtime_publisher_->msg_;
  msg.battery_states.reserve(state_joints_.size());
  for (size_t i = 0; i < state_joints_.size(); ++i)
  {
    sensor_msgs::msg::BatteryState battery;
    battery.location.reserve(MAX_LENGTH);
    battery.serial_number.reserve(MAX_LENGTH);
    msg.battery_states.emplace_back(std::move(battery));
  }
  raw_battery_states_realtime_publisher_->unlock();

  // Get count of enabeled joints for each interface
  for (size_t i = 0; i < state_joints_.size(); ++i)
  {
    const auto & interfaces = params_.interfaces.state_joints_map.at(params_.state_joints.at(i));
    const auto & battery_properties = params_.state_joints_map.at(params_.state_joints.at(i));

    if (interfaces.battery_voltage){
      counts_.voltage_cnt++;
      battery_presence_[i] = true;
    }
    if (interfaces.battery_temperature)
    {
      counts_.temperature_cnt++;
      battery_presence_[i] = true;
    }
    if (interfaces.battery_current)
    {
      counts_.current_cnt++;
      battery_presence_[i] = true;
    }
    if (interfaces.battery_charge)
    {
      counts_.charge_cnt++;
      battery_presence_[i] = true;
    }
    if (interfaces.battery_power_supply_status) battery_presence_[i] = true;
    if (interfaces.battery_power_supply_health) battery_presence_[i] = true;
    if (interfaces.battery_percentage)
    {
      counts_.percentage_cnt++;
      battery_presence_[i] = true;
    }
    else
    {
      auto min_volt = battery_properties.minimum_voltage;
      auto max_volt = battery_properties.maximum_voltage;
      if (interfaces.battery_voltage and (!std::isnan(min_volt)) and (!std::isnan(max_volt)))
      {
        if (min_volt == max_volt)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Minimum and maximum battery voltage levels can't be equal.");
          return controller_interface::CallbackReturn::ERROR;
        }
        counts_.percentage_cnt++;
      }
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size() * 7);
  for (size_t i = 0; i < state_joints_.size(); ++i)
  {
    const auto & interfaces = params_.interfaces.state_joints_map.at(params_.state_joints.at(i));
    if (interfaces.battery_voltage)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_voltage");
    }
    if (interfaces.battery_temperature)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_temperature");
    }
    if (interfaces.battery_current)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_current");
    }
    if (interfaces.battery_charge)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_charge");
    }
    if (interfaces.battery_percentage)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_percentage");
    }
    if (interfaces.battery_power_supply_status)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_power_supply_status");
    }
    if (interfaces.battery_power_supply_health)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_power_supply_health");
    }
    if (interfaces.battery_present)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_present");
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces found to publish.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // handle individual battery states initializations
  auto & raw_battery_states_msg = raw_battery_states_realtime_publisher_->msg_;
  for (size_t i = 0; i < state_joints_.size(); ++i)
  {
    auto & battery_state = raw_battery_states_msg.battery_states[i];
    const auto & battery_properties = params_.state_joints_map.at(params_.state_joints.at(i));

    battery_state.header.frame_id = state_joints_[i];
    battery_state.voltage = kUninitializedValue;
    battery_state.temperature = kUninitializedValue;
    battery_state.current = kUninitializedValue;
    battery_state.charge = kUninitializedValue;
    battery_state.capacity = static_cast<float>(battery_properties.capacity);
    battery_state.design_capacity = static_cast<float>(battery_properties.design_capacity);
    battery_state.percentage = kUninitializedValue;
    battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state.power_supply_technology = static_cast<char>(battery_properties.power_supply_technology);
    battery_state.present = battery_presence_[i];
    battery_state.cell_voltage = {};
    battery_state.cell_temperature = {};
    battery_state.location = battery_properties.location;
    battery_state.serial_number = battery_properties.serial_number;
  }

  // handle aggregate battery state initialization
  auto & battery_state_msg = battery_state_realtime_publisher_->msg_;

  battery_state_msg.voltage = kUninitializedValue;
  battery_state_msg.temperature = kUninitializedValue;
  battery_state_msg.current = kUninitializedValue;
  battery_state_msg.charge = kUninitializedValue;
  battery_state_msg.capacity = static_cast<float>(params_.state_joints_map.at(params_.state_joints.at(0)).capacity); //TODO: this is wrong bc what if the first one doesnt exist
  battery_state_msg.design_capacity = static_cast<float>(params_.state_joints_map.at(params_.state_joints.at(0)).design_capacity);
  battery_state_msg.percentage = kUninitializedValue;
  battery_state_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_state_msg.power_supply_technology = static_cast<char>(params_.state_joints_map.at(params_.state_joints.at(0)).power_supply_technology); //TODO: this is wrong bc what if the first one doesnt exist
  battery_state_msg.present = true;
  battery_state_msg.cell_voltage = {};
  battery_state_msg.cell_temperature = {};
  battery_state_msg.location = params_.state_joints_map.at(params_.state_joints.at(0)).location; //TODO: this is wrong bc what if the first one doesnt exist
  battery_state_msg.serial_number = params_.state_joints_map.at(params_.state_joints.at(0)).serial_number; //TODO: this is wrong bc what if the first one doesnt exist

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BatteryStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  sums_ = {};
  int interface_cnt = 0;

  if (raw_battery_states_realtime_publisher_ && raw_battery_states_realtime_publisher_->trylock())
  {
    auto & raw_battery_states_msg = raw_battery_states_realtime_publisher_->msg_;
    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      const auto & interfaces = params_.interfaces.state_joints_map.at(params_.state_joints.at(i));

      raw_battery_states_msg.battery_states[i].header.stamp = time;

      if (interfaces.battery_voltage)
      {

        raw_battery_states_msg.battery_states[i].voltage = get_or_nan(interface_cnt);
        sums_.voltage_sum += raw_battery_states_msg.battery_states[i].voltage;
        interface_cnt++;
      }
      if (interfaces.battery_temperature)
      {
        raw_battery_states_msg.battery_states[i].temperature = get_or_nan(interface_cnt);
        sums_.temperature_sum += raw_battery_states_msg.battery_states[i].temperature;
        interface_cnt++;
      }
      if (interfaces.battery_current)
      {
        raw_battery_states_msg.battery_states[i].current = get_or_nan(interface_cnt);
        sums_.current_sum += raw_battery_states_msg.battery_states[i].current;
        interface_cnt++;
      }
      if (interfaces.battery_charge)
      {
        raw_battery_states_msg.battery_states[i].charge = get_or_nan(interface_cnt);
        sums_.charge_sum += raw_battery_states_msg.battery_states[i].charge;
        interface_cnt++;
      }
      if (interfaces.battery_percentage)
      {
        raw_battery_states_msg.battery_states[i].percentage = get_or_nan(interface_cnt);
        sums_.percentage_sum += raw_battery_states_msg.battery_states[i].percentage;
        interface_cnt++;
      }
      else
      {
        auto min_volt = params_.state_joints_map.at(params_.state_joints.at(i)).minimum_voltage;
        auto max_volt = params_.state_joints_map.at(params_.state_joints.at(i)).maximum_voltage;
        float voltage = raw_battery_states_msg.battery_states[i].voltage;
        if (!(std::isnan(voltage) or std::isnan(min_volt) or std::isnan(max_volt)))
        {
          raw_battery_states_msg.battery_states[i].percentage = 
          static_cast<float>((voltage - min_volt) / (max_volt - min_volt));
          sums_.percentage_sum += raw_battery_states_msg.battery_states[i].percentage;
        }
      }
      if (interfaces.battery_power_supply_status)
      {
        raw_battery_states_msg.battery_states[i].power_supply_status = get_or_unknown(interface_cnt);
        interface_cnt++;
      }
      if (interfaces.battery_power_supply_health)
      {
        raw_battery_states_msg.battery_states[i].power_supply_health = get_or_unknown(interface_cnt);
        interface_cnt++;
      }
      if (interfaces.battery_present)
      {
        auto opt = state_interfaces_[interface_cnt].get_optional<bool>();
        if (opt.has_value()) raw_battery_states_msg.battery_states[i].present = static_cast<bool>(*opt);
        else raw_battery_states_msg.battery_states[i].present = false;
        interface_cnt++;
      }
    }
    raw_battery_states_realtime_publisher_->unlockAndPublish();
  }

  if (battery_state_realtime_publisher_ && battery_state_realtime_publisher_->trylock())
  {
    auto & battery_state_msg = battery_state_realtime_publisher_->msg_;

    battery_state_msg.header.stamp = time;
    if (counts_.voltage_cnt)
    {
      battery_state_msg.voltage = sums_.voltage_sum / counts_.voltage_cnt;
    }
    if (counts_.temperature_cnt)
    {
      battery_state_msg.voltage = sums_.temperature_sum / counts_.temperature_cnt;
    }
    if (counts_.current_cnt)
    {
      battery_state_msg.current = sums_.current_sum / counts_.current_cnt;
    }
    if (counts_.charge_cnt)
    {
      battery_state_msg.charge = sums_.charge_sum / counts_.charge_cnt;
    }
    if (counts_.percentage_cnt)
    {
      battery_state_msg.percentage = sums_.percentage_sum / counts_.percentage_cnt;
    }

    battery_state_realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

float BatteryStateBroadcaster::get_or_nan(int interface_cnt) {
    auto opt = state_interfaces_[interface_cnt].get_optional<double>();
    if (opt.has_value()) {
        return static_cast<float>(*opt);
    }
    return std::numeric_limits<float>::quiet_NaN();
}

char BatteryStateBroadcaster::get_or_unknown(int interface_cnt) {
    auto opt = state_interfaces_[interface_cnt].get_optional<double>();
    if (opt.has_value()) {
        return static_cast<char>(*opt);
    }
    return 0;
}

}  // namespace battery_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  battery_state_broadcaster::BatteryStateBroadcaster, controller_interface::ControllerInterface)
