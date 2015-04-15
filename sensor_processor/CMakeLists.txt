cmake_minimum_required(VERSION 2.8.3)
project(sensor_processor)

set(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge)

FIND_PACKAGE(OpenCV REQUIRED)

catkin_package()

add_executable(sensor_processor
    src/sensor_processor_node.cpp
)

target_link_libraries(sensor_processor ${catkin_LIBRARIES} ${OpenCV_LIBS})
