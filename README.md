# Turtlebot Autonomous Exploration (3D) 

## Overview

This is an implementation of infomation-theoretic exploration using [turtlebot](http://wiki.ros.org/Robots/TurtleBot) with a RGBD camera (e.g. Kinect). It is designed for autonomous mapping of indoor office-like environments (flat terrain). The output consist of both 2D and 3D [Octomap](http://octomap.github.io/) (.ot file).

All the computation is on the turtlebot laptop and intermediate results can be viewed from remote PC. 

If you find this package useful, please consider citing the follow paper:

* S. Bai, J. Wang, K. Doherty, and B. Englot, "Inference-Enabled Information-Theoretic Exploration of Continuous Action Spaces,"Proceedings of the 17th International Symposium on Robotics Research, 16 pp., September 2015. 


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

From Remote Server:
```
$ roslaunch turtlebot_exploration_3d exploration_rviz.launch
```

### Authors ###

Shi Bai, Xiangyu Xu.
[RFAL (Robust Field Autonomy Lab)](http://personal.stevens.edu/~benglot/index.html), Stevens Institute of Technology.

