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

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("source_tree");

  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory;

  factory.registerFromPlugin(loader.getOSName("delegate_action_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");

  std::string xml_file, xml_file_remote_1, remote_tree_1, remote_id_1;

  try {
    // Load the XML path from the YAML file
    std::ifstream fin(pkgpath + "/params/config.yaml");
    YAML::Node params = YAML::Load(fin);

    xml_file = pkgpath + params["tree"].as<std::string>();
    xml_file_remote_1 = pkgpath + params["remote_tree_1"].as<std::string>();
    // remote_id_1 = params["remote_id_1"].as<std::string>();

    // std::ifstream file(xml_file_remote_1);
    // std::ostringstream contents_stream;

    // contents_stream << file.rdbuf();
    // remote_tree_1 = contents_stream.str();
  } catch (YAML::Exception & e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("pkgpath", pkgpath + "/bt_xml/");

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  std::cout << "\t- Tree created from file" << std::endl;

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node);
    rate.sleep();
  }

  std::cout << "Finished" << std::endl;
  rclcpp::shutdown();
  return 0;
}
