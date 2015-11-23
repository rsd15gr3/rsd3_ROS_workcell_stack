cmake_minimum_required(VERSION 2.8.3)
project(brick_detection)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  genmsg)

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

add_service_files(
  FILES
  bricks.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(brick_detection_node src/detectBricks.h src/brick_detection_node.cpp)
add_dependencies(brick_detection_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(brick_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

