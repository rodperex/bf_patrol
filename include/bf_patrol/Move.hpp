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

#ifndef BF_PATROL__MOVE_HPP_
#define BF_PATROL__MOVE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "bf_patrol/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"


#include "datatypes.hpp"

namespace bf_patrol
{

class Move : public bf_patrol::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Waypoint>("goal")
    };
  }

private:

  std::string current_goal_; 
  std::vector<Waypoint> wps_;

};



}  // namespace bt_patrol

#endif  // BT_PATROL__MOVE_HPP_
