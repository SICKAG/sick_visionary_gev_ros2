# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

cmake_minimum_required(VERSION 3.22)
project(sick_visionary_gev_ros2 VERSION 1.1.0 LANGUAGES CXX)

# Prevent warnings in CMake>=3.24 for ExternalProject_Add()
# see https://cmake.org/cmake/help/latest/policy/CMP0135.html
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW) # valid for DOWNLOAD_EXTRACT_TIMESTAMP option in CMake 3.24 and later
endif()

# Setup project info
set(APPLICATION_NAME          "SICK Visionary GEV ROS2")
set(APPLICATION_CODENAME      "${PROJECT_NAME}")
set(APPLICATION_COPYRIGHT     "Copyright (c) 2024 SICK AG, Waldkirch")
set(APPLICATION_VENDOR_ID     "com.sick")
set(APPLICATION_VENDOR_NAME   "SICK AG")
set(APPLICATION_VENDOR_EMAIL  "techsupport3dsnapshot@sick.de")
set(APPLICATION_VENDOR_URL    "${PROJECT_HOMEPAGE_URL}")
set(APPLICATION_ID            "${APPLICATION_VENDOR_ID}.${PROJECT_NAME}")

# Check if ROS2 Humble has been sourced. Otherwise include cv_bridge.hpp
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  add_definitions(-DROS2_HUMBLE)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Setup project defaults
include(CompilerFlags)
include(BuildType)
include(InstallLocation)
include(Utils)

option(BUILD_TESTING "Build with tests" ON)
option(BUILD_COVERAGE "Build with test coverage" ON)
option(BUILD_DOCUMENTATION "Build with documentation" ON)
option(SICK_ENABLE_LOGGING "Enable printf() debug logging outputs" ON)

add_subdirectory(third_party)

if(BUILD_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Add 'Coverage' build type
  set(CMAKE_CXX_FLAGS_COVERAGE "-g -O0 --coverage")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    string(APPEND CMAKE_CXX_FLAGS_COVERAGE " -fprofile-abs-path")
  endif()
  set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "--coverage")
  set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "--coverage")
  set(CMAKE_MODULE_LINKER_FLAGS_COVERAGE "--coverage")
endif()

# We need to declare all Visionary publisher nodes on the top-level CMakeLists.txt file because
# of the following issue: https://github.com/ros2/rclcpp/issues/1362.
#
# When |rcpcpp_components_register_node| is called on a subdirectory instead of the top-level
# CMakeLists.txt it fails to register the component due to not updating the _RCPCPP_COMPONENTS
# cache variable. Therefore, we declare all publisher build targets on top-level here and add
# all further details such as sources, dependens and linked libraries in the src/ subdirectory.
find_package(rclcpp_components REQUIRED)

########## SICKPublisher Node ##########
add_library(SICKPublisher_component SHARED)
rclcpp_components_register_node(SICKPublisher_component
  PLUGIN "sick::VisionaryPublisher" EXECUTABLE visionary_publisher)

########## PointCloudPublisher Node ##########
add_library(PointCloudPublisher SHARED)
rclcpp_components_register_node(PointCloudPublisher
  PLUGIN "sick::PointCloudPublisher" EXECUTABLE pointcloud_publisher)

add_subdirectory(src)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(BUILD_DOCUMENTATION)
  add_subdirectory(doc)
endif()

# Install files.
install(DIRECTORY ${PROJECT_SOURCE_DIR}/../cti
  DESTINATION ${INSTALL_SHARE_DIR}/${PROJECT_NAME})
install(DIRECTORY launch meshes config
  DESTINATION ${INSTALL_SHARE_DIR}/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
ament_package()
