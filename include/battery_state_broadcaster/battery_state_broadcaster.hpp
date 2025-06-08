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

#ifndef BATTERY_STATE_BROADCASTER__BATTERY_STATE_BROADCASTER_HPP_
#define BATTERY_STATE_BROADCASTER__BATTERY_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "battery_state_broadcaster_parameters.hpp"
#include "control_msgs/msg/battery_states.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace battery_state_broadcaster
{
// // name constants for state interfaces
// static constexpr size_t STATE_MY_ITFS = 0;

// // name constants for command interfaces
// static constexpr size_t CMD_MY_ITFS = 0;

enum class power_supply_status : std::uint8_t
{
  POWER_SUPPLY_STATUS_UNKNOWN = 0,
  POWER_SUPPLY_STATUS_CHARGING = 1,
  POWER_SUPPLY_STATUS_DISCHARGING = 2,
  POWER_SUPPLY_STATUS_NOT_CHARGING = 3,
  POWER_SUPPLY_STATUS_FULL = 4
};
enum class power_supply_health : std::uint8_t
{
  POWER_SUPPLY_HEALTH_UNKNOWN = 0,
  POWER_SUPPLY_HEALTH_GOOD = 1,
  POWER_SUPPLY_HEALTH_OVERHEAT = 2,
  POWER_SUPPLY_HEALTH_DEAD = 3,
  POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4,
  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5,
  POWER_SUPPLY_HEALTH_COLD = 6,
  POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7,
  POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8
};
enum class power_supply_technology : std::uint8_t
{
  POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0,
  POWER_SUPPLY_TECHNOLOGY_NIMH = 1,
  POWER_SUPPLY_TECHNOLOGY_LION = 2,
  POWER_SUPPLY_TECHNOLOGY_LIPO = 3,
  POWER_SUPPLY_TECHNOLOGY_LIFE = 4,
  POWER_SUPPLY_TECHNOLOGY_NICD = 5,
  POWER_SUPPLY_TECHNOLOGY_LIMN = 6
};

/**
 * \brief Battery State Broadcaster for all or some state in a ros2_control system.
 *
 * BatteryStateBroadcaster publishes state interfaces from ros2_control as ROS messages.
 * The following state interfaces are published:
 *    <state_joint>/voltage
 *    <state_joint>/current
 *    <state_joint>/charge
 *    <state_joint>/percentage
 *    <state_joint>/power_supply_status
 *    <state_joint>/power_supply_health
 *    <state_joint>/present
 *
 * \param state_joints of the batteries to publish.
 * \param capacity of the batteries to publish.
 * \param design_capacity of the batteries to publish.
 * \param power_supply_technology of the batteries to publish.
 * \param location of the batteries to publish.
 * \param serial_number of the batteries to publish.
 *
 * Publishes to:
 *
 * - \b battery_state (sensor_msgs::msg::BatteryState): battery state of the combined battery
 * joints.
 * - \b raw_battery_states (battery_state_broadcaster::msg::BatteryStates): battery states of the
 * individual battery joints.
 *
 */
class BatteryStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  BatteryStateBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<battery_state_broadcaster::ParamListener> param_listener_;
  battery_state_broadcaster::Params params_;

  const size_t MAX_LENGTH = 64;
  float voltage_sum;
  float current_sum;
  float charge_sum;
  float percentage_sum;
  mutable float voltage_cnt = 0.0;
  mutable float current_cnt = 0.0;
  mutable float charge_cnt = 0.0;
  mutable float percentage_cnt = 0.0;

  std::vector<std::string> state_joints_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::BatteryStates>>
    raw_battery_states_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>
    battery_state_realtime_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::BatteryStates>>
    raw_battery_states_realtime_publisher_;

  std::vector<sensor_msgs::msg::BatteryState> battery_states_data_;

private:
};

}  // namespace battery_state_broadcaster

#endif  // BATTERY_STATE_BROADCASTER__BATTERY_STATE_BROADCASTER_HPP_
