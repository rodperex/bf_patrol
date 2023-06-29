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

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "yaml-cpp/yaml.h"

#include "behaviorfleets/RemoteDelegateActionNode.hpp"


int main(int argc, char * argv[])
{
  std::string params_file = "patrol_config.yaml";

  if (argc > 1) {
    params_file = std::string(argv[1]);
  }

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bf_patrol");

  std::list<std::shared_ptr<BF::RemoteDelegateActionNode>> nodes;

  try {
    std::cout << "Configuration file: " << params_file << std::endl;
    std::ifstream fin(pkgpath + "/params/" + params_file);
    YAML::Node params = YAML::Load(fin);
    int num_nodes = params["nodes"].as<int>();
    std::vector<std::string> missions = params["missions"].as<std::vector<std::string>>();

    int mission_index = 0;
    for (int i = 0; i < num_nodes; ++i) {
      std::string name = "patrol_" + std::to_string(i + 1);
      std::string type = missions[mission_index];

      auto node = std::make_shared<BF::RemoteDelegateActionNode>(name, type);
      nodes.push_back(node);
      exec.add_node(node);

      std::cout << "\n\n******** Created node " << name << " with mission " << type << "\n\n" <<
        std::endl;

      mission_index = (mission_index + 1) % missions.size();
    }

  } catch (YAML::Exception & e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    return 1;
  }

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
