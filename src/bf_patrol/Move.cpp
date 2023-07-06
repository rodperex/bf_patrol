// Copyright 2019 Intelligent Robotics Lab
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

#include "bf_patrol/Move.hpp"

namespace bf_patrol
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: bf_patrol::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
    conf)
{
}

void
Move::on_tick()
{
  getInput("goal", current_goal_);

  config().blackboard->get("waypoints", s_wps_);
  wps_ = deserialize_wps(s_wps_);

  for (auto wp:wps_) {
    if (wp.id == current_goal_) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.pose.orientation.w = 1.0;
      goal.pose.position.x = wp.x;
      goal.pose.position.y = wp.y;
      goal_.pose = goal;
      break;
    }
  }

}

BT::NodeStatus
Move::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation suceeded");

  std::vector<Waypoint>::iterator ptr;
  for (ptr = wps_.begin(); ptr != wps_.end(); ptr++) {
    if (ptr->id == current_goal_) {
      RCLCPP_INFO(node_->get_logger(), "waipoint %s tagged as visited", ptr->id.c_str());
      ptr->visited = true;
      break;
    }
  }

  s_wps_ = serialize_wps(wps_);

  config().blackboard->set("waypoints", s_wps_);

  return BT::NodeStatus::SUCCESS;
}


}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bf_patrol::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<bf_patrol::Move>(
    "Move", builder);
}
