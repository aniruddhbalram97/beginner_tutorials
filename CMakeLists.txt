cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs)
find_package(tf2)
find_package(tf2_ros)
find_package(tf2_geometry_msgs)
# publisher/subscriber executables
add_executable(talker src/publisher_member_function.cpp)
add_executable(listener src/subscriber_member_function.cpp)
# server/client executables
#add_executable(server src/add_two_ints_server.cpp)
add_executable(client src/add_two_ints_client.cpp)
# publisher/subscriber executables
ament_target_dependencies(talker rclcpp std_msgs example_interfaces geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
# server/client dependencies
#ament_target_dependencies(server rclcpp example_interfaces)
ament_target_dependencies(client rclcpp example_interfaces)
install(TARGETS
  talker
  listener
  #server
  client
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  # Add a gtest executable
  set(TESTFILES
    test/run_test.cpp
    test/main.cpp)
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs)

  install(TARGETS
  test_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()