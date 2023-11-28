cmake_minimum_required(VERSION 3.8)
project(apriltag_ros2)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# Errors with: cmake_minimum_required(VERSION 3.3) 
# CMake Error at /opt/ros/galactic/share/ament_cmake_auto/cmake/ament_auto_find_build_dependencies.cmake:64 (if):
#   if given arguments:
#     "_dep" "IN_LIST" "_ARG_REQUIRED"
#   Unknown arguments specified
# Call Stack (most recent call first):
#   CMakeLists.txt:18 (ament_auto_find_build_dependencies)

###########
## Build ##
###########
include_directories(/usr/local/include/apriltag)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

# ament_auto_add_library(common_functions SHARED
#  src/common_functions.cpp
#  include/apriltag_ros2/common_functions.h)
add_library(common_functions src/common_functions.cpp)
target_link_libraries(common_functions PUBLIC apriltag)
ament_target_dependencies(common_functions PUBLIC
 geometry_msgs
 rclcpp
 sensor_msgs
 std_msgs
 cv_bridge
 apriltag_ros2_interfaces
 OpenCV
 )
target_include_directories(common_functions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# ament_auto_add_executable(continuous_detector src/continuous_detector.cpp)
add_executable(continuous_detector
  src/continuous_detector.cpp)
target_link_libraries(continuous_detector common_functions)
# ament_target_dependencies(continuous_detector common_functions)
#############
## Install ##
#############
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME})

# install(TARGETS common_functions
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin)

# install(TARGETS continuous_detector
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME})

# ament_package()
ament_auto_package()
