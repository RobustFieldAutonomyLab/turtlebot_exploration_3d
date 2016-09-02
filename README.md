# README #

This is an implementation of infomation-theoretic autonomous exploration using turtlebot with a RGBD camera for indoor environments. All the computation is on the turtlebot laptop and intermediate results can be viewed from remote PC. The output consist of both 2D and 3D Octomap (.ot file).

If you find this package useful, please consider citing the follow paper:

S. Bai, J. Wang, K. Doherty, and B. Englot, "Inference-Enabled Information-Theoretic Exploration of Continuous Action Spaces,"Proceedings of the 17th International Symposium on Robotics Research, 16 pp., September 2015. 


* Version  0.0.1

### How do I get set up? ###

* Compiling from source:

```
my_catkin_workspace/src$ git clone
my_catkin_workspace/src$ cd ..
my_catkin_workspace$ catkin_make
```

* Running:

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
RFAL (Robust Field Autonomy Lab), Stevens Institute of Technology.
http://personal.stevens.edu/~benglot/index.html
