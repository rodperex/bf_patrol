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

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bf_patrol");
  std::string name = argv[1]; 
  std::string type = argv[2];

  auto node = std::make_shared<BF::RemoteDelegateActionNode>(name, type);
  std::cout << "\n\n******** Created node " << name << " with mission " << type << "\n\n" <<
    std::endl;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
