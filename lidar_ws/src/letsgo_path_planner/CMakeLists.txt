cmake_minimum_required(VERSION 3.8)
project(letsgo_path_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

add_library(straight_navfn_planner SHARED src/straight_navfn_planner.cpp src/straight_segment.cpp)
target_compile_features(straight_navfn_planner PRIVATE cxx_std_17)
target_include_directories(straight_navfn_planner PRIVATE include)
ament_target_dependencies(straight_navfn_planner
  geometry_msgs nav2_core nav2_costmap_2d nav2_navfn_planner nav2_util pluginlib
)
pluginlib_export_plugin_description_file(nav2_core planner_plugin.xml)
install(TARGETS straight_navfn_planner LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_straight_segment test/test_straight_segment.cpp)
  target_include_directories(test_straight_segment PRIVATE include)
  target_link_libraries(test_straight_segment straight_navfn_planner)
  ament_target_dependencies(test_straight_segment geometry_msgs nav2_costmap_2d)
endif()

ament_package()
