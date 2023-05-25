// Copyright 2023 Intelligent Robotics Lab
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
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviorfleets/RemoteDelegateActionNode.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto node_1 = std::make_shared<BF::RemoteDelegateActionNode>("patrol_1", "patrol");
  auto node_2 = std::make_shared<BF::RemoteDelegateActionNode>("patrol_2", "patrol");
  auto node_3 = std::make_shared<BF::RemoteDelegateActionNode>("patrol_3", "patrol");
  auto node_4 = std::make_shared<BF::RemoteDelegateActionNode>("patrol_4", "patrol");
  auto node_5 = std::make_shared<BF::RemoteDelegateActionNode>("patrol_5", "patrol");
  auto node_6 = std::make_shared<BF::RemoteDelegateActionNode>("patrol_6", "patrol");
  auto node_7 = std::make_shared<BF::RemoteDelegateActionNode>("patrol_7", "patrol");

  exec.add_node(node_1);
  exec.add_node(node_2);
  exec.add_node(node_3);
  exec.add_node(node_4);
  exec.add_node(node_5);
  exec.add_node(node_6);
  exec.add_node(node_7);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
