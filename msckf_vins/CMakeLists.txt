cmake_minimum_required(VERSION 2.8.3)
project(msckf_vins)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -O3 -Wall")
#SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED  COMPONENTS roscpp std_msgs geometry_msgs nav_msgs cv_bridge tf )
FIND_PACKAGE(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msckf_vins
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
   include
   ${Eigen_INCLUDE_DIRS}
)

add_executable(msckf_vins_node 
  src/main.cpp
  src/Camera.cpp
  src/math_tool.cpp
  src/MSCKF.cpp
)

target_link_libraries(msckf_vins_node
  ${catkin_LIBRARIES}
)
