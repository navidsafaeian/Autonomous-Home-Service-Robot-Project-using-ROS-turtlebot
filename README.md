[gif]:src/gif/homeServiceRobot.gif
# RoboND-Home-Service-Robot-Project
The goal of this project is to program a Home Service Robot than can autonomously map an environment and navigate to pick up and drop off virtual objects. The robot-ROS turtlebot is simulated in Gazebo environment. 

![gif]

## Summary of Tasks
1. Design a simple environment with the Building Editor in Gazebo.
2. Teleoperate your robot and manually test SLAM.
3. Create a wall_follower node that autonomously drives your robot to map your environment.
4. Use the ROS navigation stack and manually commands your robot using the 2D Nav Goal arrow in rviz to move to 2 different desired positions and orientations.
5. Write a pick_objects node that commands your robot to move to the desired pickup and drop off zones.
6. Write an add_markers node that subscribes to your robot odometry, keeps track of your robot pose, and publishes markers to rviz.


#### Update your system 
- Use `sudo apt-get update`

#### Install the ROS navigation stack
- Use `sudo apt-get install ros-kinetic-navigation`

#### The following ROS packages will be using in this project:

* **gmapping**: With the `gmapping_demo.launch` file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
- https://github.com/ros-perception/slam_gmapping.git

* **turtlebot_teleop**: With the `keyboard_teleop.launch` file, you can manually control a robot using keyboard commands.
- https://github.com/turtlebot/turtlebot.git

* **turtlebot_rviz_launchers**: With the `view_navigation.launch` file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
- https://github.com/turtlebot/turtlebot_interactions.git

* **turtlebot_gazebo**: With the `turtlebot_world.launch` you can deploy a turtlebot in a gazebo environment by linking the world file to it.
- https://github.com/turtlebot/turtlebot_simulator.git

#### How to use this repository
first change to the workspace directory
``` bash
$ cd ~/catkin_ws
```
Testing SLAM
``` bash
$ ./src/ShellScripts/test_slam.sh
```
Testing Navigation
``` bash
$ ./src/ShellScripts/test_navigation.sh
```
Saving SLAM Map
``` bash
./src/ShellScripts/save_map.sh
```
Pick_objects
``` bash
$ ./src/ShellScripts/pick_objects.sh
```
Add Markers
``` bash
$ ./src/ShellScripts/add_marker.sh
```
Home Service
``` bash
$ ./src/ShellScripts/home_service.sh
```



