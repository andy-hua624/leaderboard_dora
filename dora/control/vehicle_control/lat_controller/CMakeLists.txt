cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(lat_controller_node)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Current dir stored in variable: ${CURRENT_DIR}")
find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)
find_package(PkgConfig)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})



pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

message(${EIGEN_INCLUDE_DIRS})

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  $ENV{HOME}/disk/dora_project/dora/apis/c/node/ #dora的头文件路径 node_api.h
  $ENV{HOME}/disk/dora_project/dora/apis/c/operator
  $ENV{HOME}/disk/dora_project/hongmeng/include
  $ENV{HOME}/disk/dora_project/hongmeng/control/vehicle_control/lat_controller/include/

  #${CURRENT_DIR}/rs_driver/src #雷达的头文件路径
  $ENV{HOME}/disk/dora_project/dora/examples/c++-ros2-dataflow/build/ #C++ros的头文件路径
  
)

#  add_executable(lat_controller_node lat_test.cpp pure_pursuit.cpp)
add_executable(lat_controller_node interface_lat.cpp pure_pursuit.cpp)

target_link_libraries(lat_controller_node
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${catkin_LIBRARIES}
  # glog::glog
  $ENV{HOME}/disk/dora_project/dora/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
  pcap
)

#add_executable(lat_controller_test_node interface_lat_test.cpp pure_pursuit.cpp $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/dora-ros2-bindings.cc #$ENV{HOME}/dora/examples/c++-ros2-dataflow/build/dora-node-api.cc)
#target_link_libraries(lat_controller_test_node
#  ${PCL_LIBRARIES}
#  ${EIGEN_LIBRARIES}
#  ${YAMLCPP_LIBRARIES}
#  ${catkin_LIBRARIES}
#  # glog::glog
#  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
#  $ENV{HOME}/dora/target/debug/libdora_node_api_cxx.a
#  #$ENV{HOME}/dora/target/release/libdora_operator_api_cxx.a
#  m
#  rt
#  dl 
#  pthread
#  pcap
#)



