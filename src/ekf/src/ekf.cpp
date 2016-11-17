#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "ros/time.h"
#include <sstream>
#include <vector>
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <Eigen/Dense>
/*#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_datatypes.h"
#include <math.h> 
*/

using Eigen::MatrixXd;


class State{

  public:
    float x, y, theta;
    ros::Time time_now;

    State(float x, float y, float theta,  ros::Time time_now){
      this->x = x;
      this->y = y;
      this->theta = theta;
      this->time_now = time_now;
    } 
    State(float x, float y, float theta)
    {
      this->x = x;
      this->y = y;
      this->theta = theta;
    } 
    std::vector<float> state_vector(){
      std::vector<float> vec;
      vec.push_back(x);
      vec.push_back(y);
      vec.push_back(theta);
      return vec;
    }
};

//Global Variables
//static ros::Time begin_time = ros::Time::now();
static State* odom_data = NULL;
//nav_msgs::Odometry::ConstPtr& scan_data;
int counter_steps = 0;
State* prev_state = NULL;
State* new_state = NULL;

double get_theta(geometry_msgs::Quaternion q){
    return tf::getYaw(q);
}

float get_velocity(float new_x, float prev_x, double delta_time){
  float distance = new_x - prev_x;
  return distance / delta_time;
}

State* f(State* state, const nav_msgs::Odometry::ConstPtr& msg, ros::Time time_now){
  float x = msg->pose.pose.position.x - state->x;
  float y = msg->pose.pose.position.y - state->y;
  float theta = get_theta(msg->pose.pose.orientation);
  return new State(x,y,theta, time_now);  
}

std::vector<float> F(State prev_state, State new_state){
  std::vector<float> v; 
  double delta_time = new_state.time_now.toSec() - prev_state.time_now.toSec();
  float vx = get_velocity(new_state.x, prev_state.x, delta_time);
  float vy = get_velocity(new_state.y, prev_state.y, delta_time);
  float theta = prev_state.theta;
  float x = (-sin(theta)* vx - cos(theta) * vy)* delta_time;
  float y = (cos(theta) * vx + sin(theta) * vy) * delta_time;
  v.push_back(x);
  v.push_back(y);
  v.push_back(theta);
  return v;
}



void ekf_step(const nav_msgs::Odometry::ConstPtr& msg_odom, ros::Time time_step){
  //init in first step
  if(counter_steps = 0)
  {
    float x = msg_odom->pose.pose.position.x;
    float y = msg_odom->pose.pose.position.y;
    float theta = get_theta(msg_odom->pose.pose.orientation);
    prev_state = new State(x,y,theta, time_step);
  }
  else
  {
    //prediction Step
    new_state = f(prev_state, msg_odom, time_step);
  }
}

/*
void scan_receiver(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //TO DO: Base Scan 
}
*/

void odom_receiver(const nav_msgs::Odometry::ConstPtr& msg)
{
  printf("Seq: [%d]", msg->header.seq);
  printf("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  printf("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  printf("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  double theta = get_theta(msg->pose.pose.orientation);
  odom_data = new State(x,y,theta);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "ekf");   
  ros::NodeHandle n;
  //begin_time = ros::Time::now();   
  ros::Subscriber sub_odom = n.subscribe("odom", 1000, odom_receiver);
  //ros::Subscriber sub_scan = n.subscribe("base_scan", 1000, scan_receiver);
  //ros::Subscriber sub_map = n.subscribe("map", 1000, scan_receiver);
  ros::Publisher pub_new_estimates = n.advertise<std_msgs::String>("different_chatter", 1000);

  //ekf_step(sub_odom, sub_scan);
  //counter_steps++;

  ros::Rate loop_rate(10);   

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;
    std::cout << "End Matrix calculus" << std::endl;

    pub_new_estimates.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  ros::spin();

  return 0;
}