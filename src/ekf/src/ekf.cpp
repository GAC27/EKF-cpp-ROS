#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/time.h"
#include <sstream>
#include <vector>
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <math.h> 
/*#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_datatypes.h"
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
//ros::Time* begin_time;
State* odom_data = NULL;
//nav_msgs::Odometry::ConstPtr& scan_data;
nav_msgs::OccupancyGrid::ConstPtr occupancyGrid;
MatrixXd noiseQk(3,3);
MatrixXd covarianceK(3,3);
MatrixXd map;
int counter_steps = 0;
State* prev_state = NULL;
State* new_state = NULL;
bool Map_Active=false;

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

MatrixXd F(State prev_state, State new_state){
  std::vector<float> v; 
  double delta_time = new_state.time_now.toSec() - prev_state.time_now.toSec();
  float vx = get_velocity(new_state.x, prev_state.x, delta_time);
  float vy = get_velocity(new_state.y, prev_state.y, delta_time);
  float theta = prev_state.theta;
  float x = (-sin(theta)* vx - cos(theta) * vy)* delta_time;
  float y = (cos(theta) * vx + sin(theta) * vy)* delta_time;
  
  MatrixXd m(3,3);
  m(0,0) = 1;
  m(0,1) = 0;
  m(0,2) = 0;

  m(1,0) = 0;
  m(1,1) = 1;
  m(1,2) = 0;

  m(2,0) = x;
  m(2,1) = y;
  m(2,2) = 1;

  return m;
}


MatrixXd Covariance(State prev_state, State new_state){
  noiseQk(0,0) = 0.01;
  noiseQk(0,1) = 0;
  noiseQk(0,2) = 0;

  noiseQk(1,0) = 0;
  noiseQk(1,1) = 0.01;
  noiseQk(1,2) = 0;

  noiseQk(2,0) = 0;
  noiseQk(2,1) = 0;
  noiseQk(2,2) = 0.01;

  MatrixXd jacobi=F(prev_state,new_state);
  MatrixXd jacobiTransposed= jacobi.transpose();

  return jacobi * covarianceK * jacobiTransposed + noiseQk;

}


/*Receives a matrix n x n and a state and calculates the expected laser scan 
* The map data, in row-major order, starting with (0,0).  Occupancy
* probabilities are in the range [0,100].  Unknown is -1. */
float* raycast(State s){
  //Parameters
  float ang_incr= 0.005814; //Taken from the laserScan topic
  float scan_ang= 2.090000 * 2; //Taken from the laserScan topic
  float resolution= occupancyGrid->info.resolution;       //TODO adjust values
  float max_range=5.600000;       //TODO adjust values        

  double number_rays= ceil(scan_ang/ang_incr) + 1; 
  std::vector<float> scan;  

  float ang_start = s.theta + scan_ang / 2;
  float ang_end = s.theta - scan_ang / 2;
  

  float dist;
  float ang;
  float ind_x;
  float ind_y;

  for(int i = 1; i <= number_rays; i++){
    dist = resolution;
    ang= ang_start + (i-1)*ang_incr;

    while(true){
      ind_x = round(s.x + (occupancyGrid->info.width/2) + dist * cos(ang) * 100);
      ind_y = round(s.y + (occupancyGrid->info.height/2) + dist * sin(ang) * 100);
      if(dist > max_range){
        scan.push_back(-1);   //I defined -1 as being the value for not found 
        break;
      }

      if( map(ceil(ind_x), ceil(ind_y)) >= 70){    //if found an obstacle with 50% certainty
        scan.push_back(dist);
      
        break;
      }
      dist = dist + resolution;
    }
  } 

  return &scan[0];

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
    MatrixXd predictedCovariance = Covariance(*prev_state,*new_state);

    float* predictedObservation = raycast(*new_state);

  }
}

MatrixXd occupancyGridToMatrix(){
  MatrixXd m(occupancyGrid->info.height, occupancyGrid->info.width);

  for(int line=0; line < occupancyGrid->info.height; line++){
    for(int column=0; column < occupancyGrid->info.width; column++){
      m(line,column) = occupancyGrid->data[column + line * occupancyGrid->info.width];
    }    
  }

  return m;
}

void odom_receiver(const nav_msgs::Odometry::ConstPtr& msg)
{
  //printf("Seq: [%d]", msg->header.seq);
  //printf("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //printf("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //printf("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  double theta = get_theta(msg->pose.pose.orientation);
  odom_data = new State(x,y,theta);
}

void map_receiver(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  
  occupancyGrid = msg;
  
  map = occupancyGridToMatrix();

  Map_Active=true;
}

void scan_receiver(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(Map_Active){
    State *testState=new State(-20,-100,0);
    
    float* f=raycast(*testState);

    for(int i=0; i<20;i++){
      printf("raycast: (%d,%f)   | laser: (%d, %f)\n",i,f[i],i,msg->ranges[i]);
      
    }

    printf("\n\n\n\n\n\n");
  }
  //printf("ang_min -> [%f]\n ang_max -> [%f]\nang_inc -> [%f]\nMax_range -> [%f]\n",msg->angle_min, msg->angle_max, msg->angle_increment, msg->range_max);
}






int main(int argc, char **argv)
{

  ros::init(argc, argv, "ekf");   
  ros::NodeHandle n;
  //begin_time = ros::Time::now();   
  ros::Subscriber sub_odom = n.subscribe("odom", 1000, odom_receiver);
  ros::Subscriber sub_scan = n.subscribe("base_scan", 1000, scan_receiver);
  //ros::Subscriber sub_map = n.subscribe("map", 1000, map_receiver);
  ros::Subscriber sub_map = n.subscribe("/map_from_map_server", 1000, map_receiver);    //Has to subscribe to our moded_map_server topic in order to avoid rewriting the map

  ros::Publisher pub_new_estimates = n.advertise<std_msgs::String>("different_chatter", 1000);

  //ekf_step(sub_odom, sub_scan);
  //counter_steps++;

  ros::Rate loop_rate(10);   

  //State *testState=new State(-100,-100,1);

  int count = 0;
  while (ros::ok())
  {
    
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    if(Map_Active){
      ROS_INFO("Map has been received");
      
      //printf("Seq: [%d]", occupancyGrid->header.seq);
      //printf("info: res= [%f] \n",occupancyGrid->info.resolution);
      //printf("Pose: x-> [%f]  y-> [%f]  z-> [%f]",occupancyGrid->info.origin.position.x,occupancyGrid->info.origin.position.y,occupancyGrid->info.origin.position.z);
    }
    //ROS_INFO("%s", msg.data.c_str());
    
   

    pub_new_estimates.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  ros::spin();

  return 0;
}