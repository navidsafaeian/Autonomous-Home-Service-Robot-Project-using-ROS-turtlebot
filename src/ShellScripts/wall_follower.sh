#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/src/World/myWorld.world" &
sleep 5
#xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=/home/workspace/src/launch/gmapping.launch.xml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers  view_navigation.launch" &
sleep 5
xterm -e "rosrun wall_follower wall_follower"
