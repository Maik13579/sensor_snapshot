# Copyright 2026 Maik Knof
# Maintainer: Maik Knof <maik.knof@gmx.de>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(sensor_snapshot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_snapshot_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/processing.cpp
  src/image_processing.cpp
  src/point_cloud_processing.cpp
  src/node_settings.cpp
  src/image_snapshot.cpp
  src/point_cloud_snapshot.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${sensor_snapshot_interfaces_TARGETS}
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
rclcpp_components_register_nodes(${PROJECT_NAME}
  "sensor_snapshot::ImageSnapshot"
  "sensor_snapshot::PointCloudSnapshot"
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_processing test/test_processing.cpp)
  target_link_libraries(test_processing ${PROJECT_NAME})
  ament_add_gtest(test_history test/test_history.cpp)
  target_link_libraries(test_history ${PROJECT_NAME})
  ament_add_gtest(test_sampling test/test_sampling.cpp)
  target_link_libraries(test_sampling ${PROJECT_NAME})
  ament_add_gtest(test_components test/test_components.cpp TIMEOUT 90)
  target_link_libraries(test_components ${PROJECT_NAME})
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_components rcl_interfaces sensor_msgs builtin_interfaces sensor_snapshot_interfaces)
ament_package()
