# robotics_challenge
Robotic challenge scenarios for the Robotics and Advanced subject of UPO's Software Engineering Grade

## Installation instructions

This branch has been specifically adapted to the ROS Melodic and Noetic distributions. It has been tested in Ubuntu 18.04 and Ubuntu 20.04, respectively.

We provide you with a useful installation script that you can use for easily installing it into a ROS workspace. To use it you have to specify the target catkin workspace where the node should be installed. To do this you can use:

```
rosrun robotics_challenge install.bash
```

## How to use the challenge

The goal of the challenge is to setup the simulation environment in which your path tracking, path planning and collision avoidance modules should be used in order to guide the Turtlebot robot to a goal destination.

This configuration includes the setup of:

* A gazebo world file that has the environment where the robot is placed on.
* The spawn of a Turtlebot robot equipped with a LASER sensor and a RGBD camera.
* The localization module (AMCL) and the map that should be used as the frame to determine the navigation goals.

A convenient bash script is available for each scenario (1-3). Example:

```
$  rosrun robotics_challenge robotics_challenge_1.bash
```

## Installation for testing
For start the challenge, you only need to execute the bash script robotics_challenge_3.bash with this command:
```
$ rosrun robotics_challenge robotics_challenge_3.bash
```
RVIZ topics for scan down sampler and path planning are loaded automatically. Only need to put the 2D nav goal and enjoy.

## Solution
The robot will follow a route and, if it detects an obstacle in the way, it will recalculate the route to avoid the obstacle