cmake_minimum_required(VERSION 2.8.3)
project(octomap_slice)

set(octomap_DIR "/opt/ros/noetic/share/octomap")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  message_filters
  octomap_ros
  octomap_msgs
  roscpp
  roslib
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_geometry
    message_filters
    octomap_ros
    octomap_msgs
    octomap_server
    std_msgs
    tf2
    tf2_msgs
    tf2_ros
    tf2_geometry_msgs
)

###########
## Build ##
###########
# c++11 support required
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

# Octomap slice pkgs
add_executable(octomap_slice_node
  src/octomap_slice_node.cpp
)
add_library(octomap_slice 
  src/octomap_slice.cpp
)
add_dependencies(octomap_slice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(octomap_slice_node 
  octomap_slice
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OMPL_LIBRARIES}
)

#############
## Install ##
#############

# install nodes
install(TARGETS octomap_slice
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # test all launch files
  roslaunch_add_file_check(launch)
endif()
