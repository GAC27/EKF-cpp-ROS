#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265358979

void goalCallback(const geometry_msgs::Twist& msg)
{
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true) ;
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal ;
  
  goal.target_pose.header.frame_id = "base_link" ;
  goal.target_pose.header.stamp = ros::Time::now() ;
  
  tf::Quaternion q ;  
  q.setEuler(msg.angular.x*PI/180.0, msg.angular.y*PI/180.0, msg.angular.z*PI/180.0) ;

  goal.target_pose.pose.position.x = msg.linear.x ;
  goal.target_pose.pose.position.y = msg.linear.y ;
  goal.target_pose.pose.position.z = msg.linear.z ;
  goal.target_pose.pose.orientation.x = q.x() ;
  goal.target_pose.pose.orientation.y = q.y() ;
  goal.target_pose.pose.orientation.z = q.z() ;
  goal.target_pose.pose.orientation.w = q.w() ;
  
  ROS_INFO("Sending goal") ;
  ac.sendGoal(goal) ;

  ac.waitForResult() ;

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Waypoint reached.") ;
  else
    ROS_INFO("The base failed to reach the waypoint.") ;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_link_navigation_client") ;

  ros::NodeHandle nHandle ;
  
  ros::Subscriber sub = nHandle.subscribe("base_link_goal", 10, goalCallback);

  ros::spin();
  return 0;
}
