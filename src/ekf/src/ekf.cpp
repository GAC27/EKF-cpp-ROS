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
State* prev_odom = NULL;
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

State* f(State* prevState,State* odom_state, const nav_msgs::Odometry::ConstPtr& msg, ros::Time time_now){
  float x = msg->pose.pose.position.x - odom_state->x + prevState->x;
  float y = msg->pose.pose.position.y - odom_state->y + prevState->y;
  float theta = get_theta(msg->pose.pose.orientation) - odom_state->theta + prevState->theta;
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
  m(1,0) = 0;
  m(2,0) = 0;

  m(0,1) = 0;
  m(1,1) = 1;
  m(2,1) = 0;

  m(0,2) = x;
  m(1,2) = y;
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

MatrixXd h(std::vector<float> distances,State s){
  float ang_incr= 0.005814; //Taken from the laserScan topic
  float scan_ang= 2.090000 * 2; //Taken from the laserScan topic
  float ang_start = s.theta - scan_ang / 2;
  MatrixXd result(2,distances.size());
  float fi;
  for(int i =0 ; i < distances.size() ; i++){
    fi=ang_start + ang_incr * i;
    result(0,i) = distances[i];
    result(1,i) = fi;
  }
  return result;
}


std::vector<MatrixXd> H(MatrixXd h, State s){
  std::vector<MatrixXd> jacobi;
  for(int i = 0 ; i < h.cols() ; i++){
    MatrixXd m(2,3);
    m(0,0) = -(cos(h(1,i)) * h(0,i)) / h(0,i);
    m(0,1) = -(sin(h(1,i)) * h(0,i)) / h(0,i);
    m(0,2) = 0;



    m(1,0) = (sin(h(1,i)) * h(0,i)) / pow(h(0,i),2);
    m(1,1) = -(cos(h(1,i)) * h(0,i)) / pow(h(0,i),2);
    m(1,2) = -1;
    jacobi.push_back(m);
  }
  return jacobi;
}

MatrixXd V(MatrixXd realZ, MatrixXd predictedZ){
  MatrixXd innovation(realZ.rows(),realZ.cols());

  for(int i = 0 ; i < realZ.cols() ; i++){
    innovation(0,i) = realZ(0,i) - predictedZ(0,i);
    innovation(1,i) = realZ(1,i) - predictedZ(1,i);
  }

  return innovation;
}

//With E[Vij(k+1)Vij(k+1)T]



/*Receives a matrix n x n and a state and calculates the expected laser scan 
* The map data, in row-major order, starting with (0,0).  Occupancy
* probabilities are in the range [0,100].  Unknown is -1. */
std::vector<float> raycast(State s){
  int grid_x = (unsigned int)((s.x - occupancyGrid->info.origin.position.x) / occupancyGrid->info.resolution);
  int grid_y = (unsigned int)((s.y - occupancyGrid->info.origin.position.y) / occupancyGrid->info.resolution);

  //Parameters
  float ang_incr= 0.005814; //Taken from the laserScan topic
  float scan_ang= 2.090000 * 2; //Taken from the laserScan topic
  float resolution= occupancyGrid->info.resolution;       //TODO adjust values
  float max_range=15.600000;       //TODO adjust values        

  double number_rays= floor(scan_ang/ang_incr); 
  std::vector<float> scan;  

  float ang_start = s.theta - scan_ang / 2;
  float ang_end = s.theta + scan_ang / 2;
  

  float dist;
  float ang=ang_start;
  float ind_x;
  float ind_y;

  for(int i = 0; i < number_rays; i++){
    dist=0 - 0.125; // 0.125 is the distance between the center of the robot and the laser's positon
    ind_x = (unsigned int)((prev_odom->x - occupancyGrid->info.origin.position.x) / occupancyGrid->info.resolution);
    ind_y = (unsigned int)((prev_odom->y - occupancyGrid->info.origin.position.y) / occupancyGrid->info.resolution);
    
    while(dist <= max_range){
      if(ind_x < 0 || ind_x >= map.rows() || ind_y < 0 || ind_y >= map.cols()){
        dist = max_range + 1;
        break;
      }
      else if(map(ind_y,ind_x) >= 70){
        scan.push_back(dist);
        break;
      }
      ind_x = ind_x + cos(ang);
      ind_y = ind_y + sin(ang);
      dist = dist + resolution;
    }
    if(dist > max_range){
      scan.push_back(-1);
    }

    ang += ang_incr; 
  } 

  return scan;

}

MatrixXd prediction(const nav_msgs::Odometry::ConstPtr& msg_odom, ros::Time time_step){
  new_state = f(prev_state,prev_odom, msg_odom, time_step);
  MatrixXd predictedCovariance = Covariance(*prev_state,*new_state);

  std::vector<float> predictedObservation = raycast(*new_state);

  MatrixXd _h = h(predictedObservation, *new_state);


  return _h;
}

/*
State* update(State* new_state, double kalman_Gain, MatrixXd realZ, MatrixXd predictedZ){
  State* updatedState= new
}
*/


//Kalman Gain
//Shouldn't it be a scalar?
MatrixXd KkplusOne(MatrixXd CovarianceKplusOne,MatrixXd H,MatrixXd S_KplusOne){

  MatrixXd _KkplusOne=CovarianceKplusOne * H.transpose() * S_KplusOne.inverse();

  return _KkplusOne;
}


MatrixXd S_KplusOne(MatrixXd H, MatrixXd CovarianceKplusOne /*, double RKplusOne*/){
  MatrixXd  _S_KplusOne = H * CovarianceKplusOne * H.transpose() /*+ RKplusOne*/;

  return _S_KplusOne;
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
    MatrixXd predictedZ  = prediction(msg_odom,time_step);

    //Matching Step

    //Update Step

  }
}



MatrixXd occupancyGridToMatrix(){
  MatrixXd m(occupancyGrid->info.height, occupancyGrid->info.width);

  for(int line=0; line < occupancyGrid->info.height; line++){
    for(int column=0; column < occupancyGrid->info.width; column++){
      m(line,column) = occupancyGrid->data[column + line * occupancyGrid->info.width];
    }    
  }
  
  std::cout << "Map matrix size ---->" << m.rows() << "x" << m.cols() << std::endl;

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
  prev_odom = new State(x,y,theta);
}

void map_receiver(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  
  occupancyGrid = msg;
  
  map = occupancyGridToMatrix();

  Map_Active=true;
}


void transmit_laser_scan(std::vector<float> f, const sensor_msgs::LaserScan::ConstPtr& msg)
{

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("bla_scan", 50);
  

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser";
    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    unsigned int num_readings = floor((scan.angle_max - scan.angle_min)/scan.angle_increment);
    scan.time_increment = msg->time_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;

    scan.ranges.resize(num_readings);
    scan.ranges = f;

    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}

void scan_receiver(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(Map_Active){
    
    //State *testState=new State(grid_x,grid_y,prev_odom->theta);
    
    //printf("grid_x & y: [%d,%d]\n",grid_x,grid_y);

    std::vector<float> f=raycast(*prev_odom);

    transmit_laser_scan(f, msg);

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
