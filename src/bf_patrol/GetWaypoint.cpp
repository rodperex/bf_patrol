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
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  bool pending_wps = true;
  std::string id;
  getInput("wp_id", id);

  config().blackboard->get("waypoints", wps_);

  if (id != "recharge") {
    for (auto wp:wps_) {
      if (!wp.visited && (wp.id != "recharge")) {
        id = wp.id;
        break;
      }
    }
    pending_wps = false;
  }

  if (pending_wps) {
    setOutput("waypoint", id);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bf_patrol::GetWaypoint>("GetWaypoint");
}
