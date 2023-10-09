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

add_library(utils SHARED src/bf_patrol/utils.cpp)

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

add_library(get_components_bt_node SHARED src/bf_patrol/GetComponents.cpp)
add_library(assemble_bt_node SHARED src/bf_patrol/Assemble.cpp)
add_library(target_bt_node SHARED src/bf_patrol/TargetAccomplished.cpp)
add_library(get_location_bt_node SHARED src/bf_patrol/GetComponentLocation.cpp)
add_library(pick_up_bt_node SHARED src/bf_patrol/PickUpComponent.cpp)
add_library(release_component_bt_node SHARED src/bf_patrol/ReleaseComponent.cpp)






target_link_libraries(get_waypoint_bt_node utils)
target_link_libraries(move_bt_node utils)




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

  get_components_bt_node
  assemble_bt_node
  target_bt_node
  get_location_bt_node
  pick_up_bt_node
  release_component_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_patrol src/bt_patrol_main.cpp)
ament_target_dependencies(bt_patrol ${dependencies})

add_executable(patrol src/patrol_main.cpp)
ament_target_dependencies(patrol ${dependencies})
target_link_libraries(patrol utils yaml-cpp /home/rod/robots/ros2/bf_ws/install/behaviorfleets/lib/libblackboard_manager.so)

# add_executable(bf_patrol_wip src/bf_patrol_main_wip.cpp)
# ament_target_dependencies(bf_patrol_wip ${dependencies})
# target_link_libraries(bf_patrol_wip utils yaml-cpp /home/rod/ros2_ws/bf_ws/install/behaviorfleets/lib/libblackboard_manager.so)

add_executable(factory src/factory_main.cpp)
ament_target_dependencies(factory ${dependencies})
target_link_libraries(factory utils yaml-cpp /home/rod/robots/ros2/bf_ws/install/behaviorfleets/lib/libblackboard_manager.so)

add_executable(remote src/remote_main.cpp)
ament_target_dependencies(remote ${dependencies})
# SOLVE THIS!
target_link_libraries(remote yaml-cpp /home/rod/robots/ros2/bf_ws/install/behaviorfleets/lib/libremote_delegate_action_node.so)
# target_link_libraries(remote_tree yaml-cpp remote_delegate_action_node)

add_executable(worker src/worker_robot.cpp)
ament_target_dependencies(worker ${dependencies})
target_link_libraries(worker /home/rod/robots/ros2/bf_ws/install/behaviorfleets/lib/libremote_delegate_action_node.so)



install(TARGETS
  ${plugin_libs}
  bt_patrol
  patrol
  factory
  remote
  worker
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

install(DIRECTORY launch params config DESTINATION share/${PROJECT_NAME}/)


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
