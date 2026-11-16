cmake_minimum_required(VERSION 3.16)
project(sensor_snapshot_recorder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_snapshot_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/recorder.cpp src/endpoint.cpp src/request_options.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp rclcpp_components::component
  ${rcl_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${std_srvs_TARGETS}
  ${sensor_snapshot_interfaces_TARGETS} rosbag2_cpp::rosbag2_cpp rosbag2_storage::rosbag2_storage)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sensor_snapshot_recorder::SnapshotRecorder"
  EXECUTABLE sensor_snapshot_recorder_node)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(test_recorder test/test_recorder.cpp TIMEOUT 90)
  target_include_directories(test_recorder PRIVATE src)
  target_link_libraries(test_recorder ${PROJECT_NAME})
  target_link_libraries(test_recorder ${rosgraph_msgs_TARGETS})
  target_compile_definitions(test_recorder PRIVATE TEST_BAG_DIRECTORY="${CMAKE_CURRENT_BINARY_DIR}/test_bags")
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components rcl_interfaces sensor_msgs std_srvs
  sensor_snapshot_interfaces rosbag2_cpp rosbag2_storage)
ament_package()
