#include "navigation_utils.h"

bool goToDest(point3d go_posi, tf::Quaternion q) {

  // make an action client that spins up a thread
  MoveBaseClient ac("move_base", true);

  // cancel previous goals
  ac.cancelAllGoals();

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = go_posi.x();
  goal.target_pose.pose.position.y = go_posi.y();
  goal.target_pose.pose.position.z = go_posi.z();

  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  ROS_INFO("Sending robot to the viewpoint...");
  ac.sendGoal(goal);

  // while(ros::ok())
    ac.waitForResult(ros::Duration(120.0));

  // Returns true iff we reached the goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_utils");

  return 0;
}
