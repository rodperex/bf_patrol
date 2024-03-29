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

#include "bf_patrol/PickUpComponent.hpp"

namespace bf_patrol
{

using namespace std::chrono_literals;

PickUpComponent::PickUpComponent(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);
}

void
PickUpComponent::halt()
{
  std::cout << "PickUp halt" << std::endl;
}

BT::NodeStatus
PickUpComponent::tick()
{
  RCLCPP_DEBUG(rclcpp::get_logger("PickUpComponent"), "Picking up component");
  std::string location;
  config().blackboard->get("efbb_goal", location);

  if (location == "storage_a") {
    config().blackboard->set("efbb_piece_on_board", "A");
  } else if (location == "storage_b") {
    config().blackboard->set("efbb_piece_on_board", "B");
  }

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
    RCLCPP_INFO(
      rclcpp::get_logger("PickUpComponent"), "Component picked up at %s",
      location.c_str());
    config().blackboard->set("efbb_goal", "stacking_point");
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bf_patrol::PickUpComponent>("PickUpComponent");
}
