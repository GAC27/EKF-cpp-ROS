#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <vector>




//Global Variables
ros::Time begin_time;
nav_msgs::Odometry::ConstPtr& odom_data;
nav_msgs::Odometry::ConstPtr& scan_data;
int counter_steps = 0;


class State{
  float x, y, theta;
  ros::Time time_now;

  public:
    State(float x, float y, float theta,  ros::Time time_now){
      this.x = x;
      this.y = y;
      this.theta = theta;
      this.time_now = time_now;
    }  
    std::vector state_vector(){
      std::vector<float> vec;
      vec.push_back(x);
      vec.push_back(y);
      vec.push_back(theta);
      return vec;
    }
}

void ekf_step(const nav_msgs::Odometry::ConstPtr& msg_odom, const sensor_msgs::LaserScan::ConstPtr& msg_scan, ros::Time time_step){
  State prev_state;
  State new_state;
  //predition step
  if(counter_steps = 0)
  {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    theta = msg->pose.pose.position.x;
    prev_state = new State()
  }
}


void odom_receiver(const nav_msgs::Odometry::ConstPtr& msg)
{
  printf("Seq: [%d]", msg->header.seq);
  printf("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  printf("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  printf("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  odom_data = msg;
}


void scan_receiver(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //TO DO: Base Scan 
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ekf");   
  ros::NodeHandle n;
  begin_time = ros::Time::now();   
  ros::Subscriber sub_odom = n.subscribe("odom", 1000, odom_receiver);
  ros::Subscriber sub_scan = n.subscribe("base_scan", 1000, scan_receiver);
  //ros::Subscriber sub_map = n.subscribe("map", 1000, scan_receiver);
  ros::Publisher pub_new_estimates = n.advertise<std_msgs::String>("different_chatter", 1000);

  ekf_step(sub_odom, sub_scan);
  counter_steps++;

  ros::Rate loop_rate(10);   

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  ros::spin();

  return 0;
}