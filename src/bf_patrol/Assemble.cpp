// Copyright 2021 Intelligent Robotics Lab
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

#include "bf_patrol/Assemble.hpp"

namespace bf_patrol
{

using namespace std::chrono_literals;

Assemble::Assemble(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);
}

void
Assemble::halt()
{
  std::cout << "Assemble halt" << std::endl;
}

BT::NodeStatus
Assemble::tick()
{
  RCLCPP_DEBUG(rclcpp::get_logger("Assemble"), "Assembling product");
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = 0.5;
  vel_pub_->publish(vel_msgs);

  auto elapsed = node_->now() - start_time_;

  if (elapsed < 2s) {
    return BT::NodeStatus::RUNNING;
  } else {
    int n_assembled, target;
    config().blackboard->get("n_assembled", n_assembled);
    config().blackboard->get("assembly_target", target);
    n_assembled++;
    if (n_assembled == target) {
      RCLCPP_INFO(rclcpp::get_logger("Assemble"), "ALL products assembled");
      config().blackboard->set("products_ready", true);
    } else {
      config().blackboard->set("products_ready", false);
    }
    RCLCPP_INFO(rclcpp::get_logger("Assemble"), "Product assembled: %d done", n_assembled);
    config().blackboard->set("n_assembled", n_assembled);
    config().blackboard->set("efbb_goal", "waiting_point");
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bf_patrol::Assemble>("Assemble");
}
