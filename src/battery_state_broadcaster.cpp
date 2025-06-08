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

BatteryStateBroadcaster::BatteryStateBroadcaster() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<battery_state_broadcaster::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  state_joints_ = params_.state_joints;

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
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
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
    battery.serial_number.reserve(MAX_LENGTH);
    battery.location.reserve(MAX_LENGTH);
    msg.battery_states.emplace_back(std::move(battery));
  }
  raw_battery_states_realtime_publisher_->unlock();

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
      voltage_cnt++;
    }
    if (interfaces.battery_current)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_current");
      current_cnt++;
    }
    if (interfaces.battery_charge)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_charge");
      charge_cnt++;
    }
    if (interfaces.battery_percentage)
    {
      state_interfaces_config.names.push_back(state_joints_[i] + "/battery_percentage");
      percentage_cnt++;
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
  battery_states_data_.resize(state_joints_.size());

  auto & battery_state_msg = battery_state_realtime_publisher_->msg_;

  battery_state_msg.voltage = kUninitializedValue;
  battery_state_msg.current = kUninitializedValue;
  battery_state_msg.charge = kUninitializedValue;
  battery_state_msg.capacity = static_cast<float>(params_.capacity);
  battery_state_msg.design_capacity = static_cast<float>(params_.design_capacity);
  battery_state_msg.percentage = kUninitializedValue;
  battery_state_msg.power_supply_status = kUninitializedValue;
  battery_state_msg.power_supply_health = kUninitializedValue;
  battery_state_msg.power_supply_technology = static_cast<char>(params_.power_supply_technology);
  battery_state_msg.cell_voltage = {};  // TODO(yara): fill with actual cell voltages
  battery_state_msg.location = params_.location;
  battery_state_msg.serial_number = params_.serial_number;

  auto & raw_battery_states_msg = raw_battery_states_realtime_publisher_->msg_;
  for (size_t i = 0; i < state_joints_.size(); ++i)
  {
    raw_battery_states_msg.battery_states[i].voltage = kUninitializedValue;
    raw_battery_states_msg.battery_states[i].current = kUninitializedValue;
    raw_battery_states_msg.battery_states[i].charge = kUninitializedValue;
    raw_battery_states_msg.battery_states[i].capacity = static_cast<float>(params_.capacity);
    raw_battery_states_msg.battery_states[i].design_capacity =
      static_cast<float>(params_.design_capacity);
    raw_battery_states_msg.battery_states[i].percentage = kUninitializedValue;
    raw_battery_states_msg.battery_states[i].power_supply_status = kUninitializedValue;
    raw_battery_states_msg.battery_states[i].power_supply_health = kUninitializedValue;
    raw_battery_states_msg.battery_states[i].power_supply_technology =
      static_cast<char>(params_.power_supply_technology);
    raw_battery_states_msg.battery_states[i]
      .cell_voltage = {};  // TODO(yara): fill with actual cell voltages
    raw_battery_states_msg.battery_states[i].location = params_.location;
    raw_battery_states_msg.battery_states[i].serial_number = params_.serial_number;
  }

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
  voltage_sum = 0.0;
  current_sum = 0.0;
  charge_sum = 0.0;
  percentage_sum = 0.0;

  int cnt = 0;
  for (size_t i = 0; i < state_joints_.size(); ++i)
  {
    const auto & interfaces = params_.interfaces.state_joints_map.at(params_.state_joints.at(i));

    if (interfaces.battery_voltage)
    {
      battery_states_data_[i].voltage = static_cast<float>(state_interfaces_[cnt].get_value());
      voltage_sum += battery_states_data_[i].voltage;
      cnt++;
    }
    else
    {
      battery_states_data_[i].voltage = kUninitializedValue;
    }
    if (interfaces.battery_current)
    {
      battery_states_data_[i].current = static_cast<float>(state_interfaces_[cnt].get_value());
      current_sum += battery_states_data_[i].current;
      cnt++;
    }
    else
    {
      battery_states_data_[i].current = kUninitializedValue;
    }
    if (interfaces.battery_charge)
    {
      battery_states_data_[i].charge = static_cast<float>(state_interfaces_[cnt].get_value());
      charge_sum += battery_states_data_[i].charge;
      cnt++;
    }
    else
    {
      battery_states_data_[i].charge = kUninitializedValue;
    }
    if (interfaces.battery_percentage)
    {
      battery_states_data_[i].percentage = static_cast<float>(state_interfaces_[cnt].get_value());
      percentage_sum += battery_states_data_[i].percentage;
      cnt++;
    }
    else
    {
      battery_states_data_[i].percentage = kUninitializedValue;
    }
    if (interfaces.battery_power_supply_status)
    {
      battery_states_data_[i].power_supply_status =
        static_cast<char>(state_interfaces_[cnt].get_value());
      cnt++;
    }
    else
    {
      battery_states_data_[i].power_supply_status = 0;
    }
    if (interfaces.battery_power_supply_health)
    {
      battery_states_data_[i].power_supply_health =
        static_cast<char>(state_interfaces_[cnt].get_value());
      cnt++;
    }
    else
    {
      battery_states_data_[i].power_supply_health = 0;
    }
    if (interfaces.battery_present)
    {
      battery_states_data_[i].present = static_cast<bool>(state_interfaces_[cnt].get_value());
      cnt++;
    }
    else
    {
      battery_states_data_[i].present = 0;
    }
  }

  if (battery_state_realtime_publisher_ && battery_state_realtime_publisher_->trylock())
  {
    auto & battery_state_msg = battery_state_realtime_publisher_->msg_;

    battery_state_msg.header.stamp = time;
    if (voltage_cnt)
    {
      battery_state_msg.voltage = voltage_sum / voltage_cnt;
    }
    else
    {
      battery_state_msg.voltage = kUninitializedValue;
    }
    if (current_cnt)
    {
      battery_state_msg.current = current_sum / current_cnt;
    }
    else
    {
      battery_state_msg.current = kUninitializedValue;
    }
    if (charge_cnt)
    {
      battery_state_msg.charge = charge_sum / charge_cnt;
    }
    else
    {
      battery_state_msg.charge = kUninitializedValue;
    }
    if (percentage_cnt)
    {
      battery_state_msg.percentage = percentage_sum / percentage_cnt;
    }
    else
    {
      battery_state_msg.percentage = kUninitializedValue;
    }
    battery_state_msg.power_supply_status = 0;
    battery_state_msg.power_supply_health = 0;

    battery_state_realtime_publisher_->unlockAndPublish();
  }

  if (raw_battery_states_realtime_publisher_ && raw_battery_states_realtime_publisher_->trylock())
  {
    auto & raw_battery_states_msg = raw_battery_states_realtime_publisher_->msg_;
    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      raw_battery_states_msg.battery_states[i].header.stamp = time;
      raw_battery_states_msg.battery_states[i].voltage = battery_states_data_[i].voltage;
      raw_battery_states_msg.battery_states[i].current = battery_states_data_[i].current;
      raw_battery_states_msg.battery_states[i].charge = battery_states_data_[i].charge;
      raw_battery_states_msg.battery_states[i].percentage = battery_states_data_[i].percentage;
      raw_battery_states_msg.battery_states[i].power_supply_status =
        battery_states_data_[i].power_supply_status;
      raw_battery_states_msg.battery_states[i].power_supply_health =
        battery_states_data_[i].power_supply_health;
      raw_battery_states_msg.battery_states[i]
        .cell_voltage = {};  // TODO(yara): fill with actual cell voltages
      raw_battery_states_msg.battery_states[i].location = state_joints_[i];
    }
    raw_battery_states_realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace battery_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  battery_state_broadcaster::BatteryStateBroadcaster, controller_interface::ControllerInterface)
