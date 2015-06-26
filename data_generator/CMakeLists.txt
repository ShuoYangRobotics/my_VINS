cmake_minimum_required(VERSION 2.8.3)
project(data_generator)

set(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -O3 -Wall")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
   include
   ${Eigen_INCLUDE_DIRS}
)

add_executable(data_generator
    src/data_generator_node.cpp 
    src/data_generator.cpp
)

target_link_libraries(data_generator ${catkin_LIBRARIES} ${OpenCV_LIBS})
