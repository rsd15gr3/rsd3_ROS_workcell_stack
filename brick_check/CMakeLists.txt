cmake_minimum_required(VERSION 2.8.3)
project(brick_check)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

find_package(catkin REQUIRED
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

find_package(
  OpenCV REQUIRED
)

add_service_files(
  FILES
  check_brick.srv
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
)

add_executable(brick_check_node src/brick_check_node.cpp)
add_dependencies(brick_check_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(brick_check_node
  ${catkin_LIBRARIES}
)
