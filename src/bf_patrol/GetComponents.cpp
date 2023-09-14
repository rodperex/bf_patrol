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


#include "bf_patrol/GetComponents.hpp"

namespace bf_patrol
{


GetComponents::GetComponents(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
}

void
GetComponents::halt()
{
}

BT::NodeStatus
GetComponents::tick()
{
  int n_a, n_b;
  
  config().blackboard->get("n_pieces_a", n_a);
  config().blackboard->get("n_pieces_b", n_b);

  if ((n_a > 0) && (n_b > 0)) {
    n_a--;
    n_b--;
    config().blackboard->set("n_pieces_a", n_a);
    config().blackboard->set("n_pieces_a", n_b);
    config().blackboard->set("efbb_goal", "assembly_point");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bf_patrol

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bf_patrol::GetComponents>("GetComponents");
}
