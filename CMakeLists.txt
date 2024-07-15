cmake_minimum_required(VERSION 3.8)
project(composable_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(composable_nodes SHARED
            src/publisher_component.cpp
            src/subscriber_component.cpp)

target_compile_definitions(composable_nodes
  PRIVATE "COMPOSABLE_NODES_DLL")
ament_target_dependencies(composable_nodes rclcpp rclcpp_components std_msgs)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(composable_publisher src/minimal_publisher.cpp)
target_link_libraries(composable_publisher composable_nodes)
ament_target_dependencies(composable_publisher
  rclcpp)

add_executable(composable_subscriber src/minimal_subscriber.cpp)
target_link_libraries(composable_subscriber composable_nodes)
ament_target_dependencies(composable_subscriber
  rclcpp)

# add_executable(composable_node src/composable_node.cpp)
# target_link_libraries(composable_node composable_nodes)
# ament_target_dependencies(composable_node rclcpp class_loader)

install(TARGETS
  composable_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composable_publisher
  composable_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_publisher_subscriber test/test_publisher_subscriber.cpp)
  target_include_directories(test_publisher_subscriber PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_publisher_subscriber rclcpp std_msgs)
  target_link_libraries(test_publisher_subscriber composable_nodes)
endif()

ament_package()

