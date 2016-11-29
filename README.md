# Turtlebot Autonomous Exploration (3D) 

## Overview

This is an ROS implementation of infomation-theoretic exploration using [turtlebot](http://wiki.ros.org/Robots/TurtleBot) with a RGBD camera (e.g. Kinect). It is designed for autonomous mapping of indoor office-like environments (flat terrain). All the computation is performed on the turtlebot laptop and intermediate results can be viewed from remote PC. The output consist of both 2D and 3D [Octomap](http://octomap.github.io/) (.ot) file and saved on the turtlebot laptop.
Link to [wiki page](http://wiki.ros.org/turtlebot_exploration_3d) (where you can find a video example.).


If you find this package useful, please consider citing the follow paper:

* S. Bai, J. Wang, F. Chen, and B. Englot, "Information-Theoretic Exploration with Bayesian Optimization," IEEE/RSJ International Conference on Intelligent Robots and Systems(IROS), October 2016. [PDF](http://personal.stevens.edu/~benglot/Bai_Wang_Chen_Englot_IROS2016_AcceptedVersion.pdf)


## How do I get set up? 

### Download from source:

```
my_catkin_workspace/src$ git clone 
my_catkin_workspace/src$ cd ..
my_catkin_workspace$ catkin_make
```

### Configure Network:
Please follow the [turtlebot network configuration](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration) to setup.

### Running:

From Turtlebot:
```
$ roslaunch turtlebot_exploration_3d minimal_explo.launch
$ roslaunch turtlebot_exploration_3d turtlebot_gmapping.launch
$ rosrun turtlebot_exploration_3d turtlebot_exploration_3d
```
* note: The octomap will be saved to the place where you do the "rosrun".

From Remote Server:
```
$ roslaunch turtlebot_exploration_3d exploration_rviz.launch
```

### Authors ###

Shi Bai, Xiangyu Xu.
[RFAL (Robust Field Autonomy Lab)](http://personal.stevens.edu/~benglot/index.html), Stevens Institute of Technology.

