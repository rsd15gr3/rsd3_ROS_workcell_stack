cmake_minimum_required(VERSION 2.8.3)
project(brick_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msgs
  brick_detection
)

catkin_package(
  CATKIN_DEPENDS
    msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(brick_client_node src/brick_client_node.cpp)
add_dependencies(brick_client_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(brick_client_node
  ${catkin_LIBRARIES}
)

# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
