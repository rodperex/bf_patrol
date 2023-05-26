cmake_minimum_required(VERSION 3.5)
project(bf_patrol)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviorfleets REQUIRED)
find_package(bf_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    behaviortree_cpp
    action_msgs
    lifecycle_msgs
    geometry_msgs
    sensor_msgs
    nav2_msgs
    ament_index_cpp
    behaviorfleets
    bf_msgs
)


include_directories(
  include
  # ${ament_INCLUDE_DIRS}
  # ${CMAKE_CURRENT_SOURCE_DIR}/../../install/behaviorfleets/include
)

add_library(recharge_bt_node SHARED src/bf_patrol/Recharge.cpp)
add_library(patrol_bt_node SHARED src/bf_patrol/Patrol.cpp)
add_library(move_bt_node SHARED src/bf_patrol/Move.cpp)
add_library(get_waypoint_bt_node SHARED src/bf_patrol/GetWaypoint.cpp)
add_library(battery_checker_bt_node SHARED src/bf_patrol/BatteryChecker.cpp)
add_library(track_objects_bt_node SHARED src/bf_patrol/TrackObjects.cpp)

add_library(forward_bt_node SHARED src/bf_patrol/Forward.cpp)
add_library(back_bt_node SHARED src/bf_patrol/Back.cpp)
add_library(turn_bt_node SHARED src/bf_patrol/Turn.cpp)
add_library(is_obstacle_bt_node SHARED src/bf_patrol/IsObstacle.cpp)

list(APPEND plugin_libs
  recharge_bt_node
  patrol_bt_node
  move_bt_node
  get_waypoint_bt_node
  battery_checker_bt_node
  track_objects_bt_node

  forward_bt_node
  back_bt_node
  turn_bt_node
  is_obstacle_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(source_tree src/source_tree_main.cpp)
ament_target_dependencies(source_tree ${dependencies})
target_link_libraries(source_tree yaml-cpp)

add_executable(remote_tree src/remote_tree_main.cpp)
ament_target_dependencies(remote_tree ${dependencies})
# SOLVE THIS!
target_link_libraries(remote_tree /home/rod/ros2_ws/bf_patrol/install/behaviorfleets/lib/libremote_delegate_action_node.so)
# target_link_libraries(remote_tree remote_delegate_action_node)


install(TARGETS
  ${plugin_libs}
  source_tree
  remote_tree
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()
