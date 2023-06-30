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

#include <string>
#include <iostream>
#include <vector>

#include "bf_patrol/GetWaypoint.hpp"

#include "behaviortree_cpp/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bf_patrol
{

int GetWaypoint::current_ = 0;

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  config().blackboard->get("waypoints", wps_);

  for(auto wp:wps_) {
    if(wp.id == "recharge") {
      recharge_point_.header.frame_id = "map";
      recharge_point_.pose.orientation.w = 1.0;
      recharge_point_.pose.position.x = wp.x;
      recharge_point_.pose.position.y = wp.y;
    } else {
      geometry_msgs::msg::PoseStamped wp_msg;
      wp_msg.header.frame_id = "map";
      wp_msg.pose.orientation.w = 1.0;
      wp_msg.pose.position.x = wp.x;
      wp_msg.pose.position.y = wp.y;
      waypoints_.push_back(wp_msg);
    }
  }

  // geometry_msgs::msg::PoseStamped wp;
  // wp.header.frame_id = "map";
  // wp.pose.orientation.w = 1.0;

  // // recharge wp
  // wp.pose.position.x = 3.67;
  // wp.pose.position.y = -0.24;
  // recharge_point_ = wp;

  // // wp1
  // wp.pose.position.x = 1.07;
  // wp.pose.position.y = -12.38;
  // waypoints_.push_back(wp);

  // // wp2
  // wp.pose.position.x = -5.32;
  // wp.pose.position.y = -8.85;
  // waypoints_.push_back(wp);

  // // wp3
  // wp.pose.position.x = -0.56;
  // wp.pose.position.y = 0.24;
  // waypoints_.push_back(wp);

  // // recharge wp
  // wp.pose.position.x = -1.7;
  // wp.pose.position.y = -0.5;
  // recharge_point_ = wp;

  // // wp1
  // wp.pose.position.x = 1.42;
  // wp.pose.position.y = -1.14;
  // waypoints_.push_back(wp);

  // // wp2
  // wp.pose.position.x = 1.58;
  // wp.pose.position.y = 1.28;
  // waypoints_.push_back(wp);

  // // wp3
  // wp.pose.position.x = -1.18;
  // wp.pose.position.y = 1.61;
  // waypoints_.push_back(wp);
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  std::string id;
  getInput("wp_id", id);

  if (id == "recharge") {
    setOutput("waypoint", recharge_point_);
  } else {
    setOutput("waypoint", waypoints_[current_++]);
    current_ = current_ % waypoints_.size();
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bf_patrol::GetWaypoint>("GetWaypoint");
}
