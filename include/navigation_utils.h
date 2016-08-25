#ifndef _NAVIGATION_UTILS_H_
#define _NAVIGATION_UTILS_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <octomap/octomap.h>
#include <tf/transform_listener.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef octomap::point3d point3d;

bool goToDest(point3d go_posi, tf::Quaternion q);

#endif
