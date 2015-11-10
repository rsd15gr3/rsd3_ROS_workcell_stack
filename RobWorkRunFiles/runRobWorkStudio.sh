#!/bin/bash
#Make sure the catkin_ws is sourced
source ../../devel/setup.sh
#Launch the robot camera node in a new terminal (also start roscore if it not running)
#xterm -e roslaunch robot_camera_launch robo_cam.launch & #brick detection launched the camera
xterm -e roslaunch brick_detection brick_detection.launch &
#Sleep a bit (Roscore need to start before we proceed)
sleep 5s
#Launch rest of the nodes in new terminals
xterm -e roslaunch kuka_ros kukaros.launch &
xterm -e roslaunch ROSNodes/wsg_50_tcp.launch &
#Wait a bit (let rest of the nodes start before we start RobWorkStudio)
sleep 2s
#Start robworkstudioand open the SceneKu6 (also locates RobWorkStudio.ini, and starts the plug in defined there)
${RWS_ROOT}/bin/release/RobWorkStudio "../KR6RSDWorkCell/SceneKu6.wc.xml"
