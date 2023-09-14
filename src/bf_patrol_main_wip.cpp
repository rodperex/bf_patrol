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
#include <fstream>
#include <ctime>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"
#include "behaviorfleets/BlackboardManager.hpp"
#include "bf_patrol/utils.hpp"

/*
This program manages the execution of a patrol mission. It loads the XML file with the configuration,
fills the blackboard with the waypoints and the package path, and executes the source tree, which, in
this case has a single action node, the delegate action node, that will delegate tasks to remote robots.
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("source_patrol_tree");

  node->declare_parameter("source_tree", "");
  node->declare_parameter("waypoints", std::vector<std::string>({}));

  std::string source_tree;
  std::vector<std::string> wps;

  node->get_parameter("source_tree", source_tree);
  node->get_parameter("waypoints", wps);

  for (auto wp : wps) {
    std::cout << "wp: " << wp << std::endl;
  }

  if (source_tree == "") {
    RCLCPP_ERROR(node->get_logger(), "Source tree VOID");
    return -1;
  }

  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  factory.registerFromPlugin(loader.getOSName("delegate_action_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bf_patrol");

  std::string xml_file;
  xml_file = pkgpath + source_tree;
   

  // std::string s_wps = serialize_wps(wps);
  std::string s_wps = "";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("pkgpath", pkgpath + "/bt_xml/");
  blackboard->set("waypoints", s_wps);

  auto bb_manager = std::make_shared<BF::BlackboardManager>(blackboard);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  BT::NodeStatus status;
  bool finish = false;

  clock_t start = clock();
  while (!finish && rclcpp::ok()) {
    status = tree.rootNode()->executeTick();
    finish = status != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node);
    rclcpp::spin_some(bb_manager);
  }
  clock_t stop = clock();
  

  if (status == BT::NodeStatus::SUCCESS) {
    std::cout << "Finished: SUCCESS" << std::endl;
  } else {
    std::cout << "Finished: FAILURE" << std::endl;
  }

  double duration = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
  std::cout << "Execution time: " << duration << " seconds" << std::endl;

  rclcpp::shutdown();
  return 0;
}
