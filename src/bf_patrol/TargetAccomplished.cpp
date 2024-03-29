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

#include "bf_patrol/TargetAccomplished.hpp"

namespace bf_patrol
{

using namespace std::chrono_literals;
using namespace std::placeholders;

TargetAccomplished::TargetAccomplished(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
}

BT::NodeStatus
TargetAccomplished::tick()
{
  RCLCPP_INFO(rclcpp::get_logger("TargetAccomplished"), "Target accomplished?");
  int n_assembled, target, n_a, n_b;
  bool products_ready;
  config().blackboard->get("n_assembled", n_assembled);
  config().blackboard->get("assembly_target", target);
  config().blackboard->get("n_pieces_a", n_a);
  config().blackboard->get("n_pieces_b", n_b);
  config().blackboard->get("products_ready", products_ready);

  if (products_ready) {
    RCLCPP_INFO(rclcpp::get_logger("TargetAccomplished"), "Yep! (%d/%d)", n_assembled, target);
    std::string id;
    config().blackboard->get("efbb_robot_id", id);
    config().blackboard->set("efbb_goal", "waiting_point_" + id);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("TargetAccomplished"), "Nope! (%d/%d)", n_assembled, target);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bf_patrol::TargetAccomplished>("TargetAccomplished");
}
