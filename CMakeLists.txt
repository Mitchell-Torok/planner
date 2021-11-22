cmake_minimum_required(VERSION 3.5)
project(planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(comp3431_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)


include_directories(include)
#add_library(planner_node src/planner_node.cpp)
#ament_target_dependencies(planner_node
#  rclcpp
#  comp3431_interfaces)
#target_link_libraries(planner_node planner)



add_executable(planner 
		src/planner_node.cpp
		src/plan_parser.cpp)
#target_link_libraries(planner planner_node)
ament_target_dependencies(planner
  rclcpp
  rclcpp_action
  nav2_msgs
  comp3431_interfaces)

install(TARGETS
  planner
  DESTINATION lib/${PROJECT_NAME})
  
#install(TARGETS
#  planner
#  DESTINATION lib/${PROJECT_NAME})
#install(DIRECTORY include/
#  DESTINATION include/)
#install(
#  TARGETS planner_node
#  EXPORT export_${PROJECT_NAME}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)


ament_package()
