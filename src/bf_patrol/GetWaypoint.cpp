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


#include "bf_patrol/GetWaypoint.hpp"

namespace bf_patrol
{

int GetWaypoint::current_ = 0;

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  bool pending_wps = false;
  std::string id;
  getInput("wp_id", id);

  config().blackboard->get("waypoints", s_wps_);

  wps_ = deserialize_wps(s_wps_);

  std::vector<Waypoint>::iterator ptr;
  ptr = wps_.begin();

  if (id != "recharge") {
    while (ptr != wps_.end() && !pending_wps) {
      RCLCPP_INFO(rclcpp::get_logger("GetWaypoint"), "Analyzing: %s", ptr->id.c_str());
      if (!ptr->visited && (ptr->id != "recharge")) {
        RCLCPP_INFO(rclcpp::get_logger("GetWaypoint"), "WP %s set as goal", ptr->id.c_str());
        id = ptr->id;
        pending_wps = true;
      }
      ++ptr;
    }
  } else {
    pending_wps = true;
  }

  RCLCPP_INFO(rclcpp::get_logger("GetWaypoint"), "ID: %s", id.c_str());

  if (pending_wps) {
    RCLCPP_INFO(rclcpp::get_logger("GetWaypoint"), "WP %s set as goal in the bb", id.c_str());
    setOutput("waypoint", id);
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_INFO(rclcpp::get_logger("GetWaypoint"), "No more WPs to visit");
  return BT::NodeStatus::FAILURE;
}


}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bf_patrol::GetWaypoint>("GetWaypoint");
}
