CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(amber_vins)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -O3 -Wall")
#SET(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs cv_bridge tf)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(OpenCv REQUIRED)

CATKIN_PACKAGE(
)

INCLUDE_DIRECTORIES(
   src
   ${Eigen_INCLUDE_DIRS}
   ${OpenCv_INCLUDE_DIRS}
)

LIST(APPEND SOURCEFILES
  src/odometry.cpp
  src/data_generator.cpp
  src/calib.cpp
  src/camera.cpp
  src/utils.cpp
)

ADD_EXECUTABLE(mixed_node
  src/mixed_node.cpp
  ${SOURCEFILES}
)
TARGET_LINK_LIBRARIES(mixed_node
  ${catkin_LIBRARIES}
)

ADD_EXECUTABLE(amber_vins_node
  src/amber_vins_node.cpp
  ${SOURCEFILES}
)

TARGET_LINK_LIBRARIES(amber_vins_node
  ${catkin_LIBRARIES}
)

ADD_EXECUTABLE(data_generator_node
  src/data_generator_node.cpp
  ${SOURCEFILES}
)

TARGET_LINK_LIBRARIES(data_generator_node
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_camera 
  test/test_camera.cpp 
  ${SOURCEFILES}) 

target_link_libraries(test_camera 
  ${catkin_LIBRARIES} 
  ${OpenCv_LIBS})
