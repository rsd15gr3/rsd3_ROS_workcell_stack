cmake_minimum_required(VERSION 2.8.3)
project(kuka_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

add_service_files(
	FILES
	getConfiguration.srv
	setConfiguration.srv
	getQueueSize.srv
	stopRobot.srv
	getIsMoving.srv
	getSafety.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/kukaROS.cpp src/PracticalSocket.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
