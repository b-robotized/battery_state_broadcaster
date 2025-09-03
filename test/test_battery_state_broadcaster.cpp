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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test/test_battery_state_broadcaster.hpp"

using dummy_package_namespace::CMD_MY_ITFS;
using dummy_package_namespace::control_mode_type;
using dummy_package_namespace::STATE_MY_ITFS;

class DummyClassNameTest : public DummyClassNameFixture<TestableDummyClassName>
{
};

TEST_F(DummyClassNameTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(battery_state_broadcaster_->params_.joints.empty());
  ASSERT_TRUE(battery_state_broadcaster_->params_.state_joints.empty());
  ASSERT_TRUE(battery_state_broadcaster_->params_.interface_name.empty());

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(battery_state_broadcaster_->params_.joints, testing::ElementsAreArray(joint_names_));
  ASSERT_TRUE(battery_state_broadcaster_->params_.state_joints.empty());
  ASSERT_THAT(battery_state_broadcaster_->state_joints_, testing::ElementsAreArray(joint_names_));
  ASSERT_EQ(battery_state_broadcaster_->params_.interface_name, interface_name_);
}

TEST_F(DummyClassNameTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = battery_state_broadcaster_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  auto state_intefaces = battery_state_broadcaster_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }
}

TEST_F(DummyClassNameTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = battery_state_broadcaster_->input_ref_.readFromNonRT();
  EXPECT_EQ((*msg)->displacements.size(), joint_names_.size());
  for (const auto & cmd : (*msg)->displacements)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }
  EXPECT_EQ((*msg)->velocities.size(), joint_names_.size());
  for (const auto & cmd : (*msg)->velocities)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }

  ASSERT_TRUE(std::isnan((*msg)->duration));
}

TEST_F(DummyClassNameTest, update_success)
{
  SetUpController();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(DummyClassNameTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(DummyClassNameTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 101.101);
}

TEST_F(DummyClassNameTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(battery_state_broadcaster_->get_node()->get_node_base_interface());

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 101.101);

  publish_commands();
  ASSERT_TRUE(battery_state_broadcaster_->wait_for_commands(executor));

  ASSERT_EQ(
    battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[CMD_MY_ITFS], 0.45);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.set_point, 0.45);
}

TEST_F(DummyClassNameTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(battery_state_broadcaster_->params_.joints.empty());
  ASSERT_TRUE(battery_state_broadcaster_->params_.state_joints.empty());
  ASSERT_TRUE(battery_state_broadcaster_->params_.interface_name.empty());

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(battery_state_broadcaster_->params_.joints, testing::ElementsAreArray(joint_names_));
  ASSERT_THAT(
    battery_state_broadcaster_->params_.state_joints,
    testing::ElementsAreArray(state_joint_names_));
  ASSERT_THAT(
    battery_state_broadcaster_->state_joints_, testing::ElementsAreArray(state_joint_names_));
  ASSERT_EQ(battery_state_broadcaster_->params_.interface_name, interface_name_);
}

TEST_F(DummyClassNameTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = battery_state_broadcaster_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  auto state_intefaces = battery_state_broadcaster_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], state_joint_names_[i] + "/" + interface_name_);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
