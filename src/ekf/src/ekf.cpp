#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Quaternion.h"
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

    State(MatrixXd stateVector)
    {
      this->x = stateVector(0,0);
      this->y = stateVector(1,0);
      this->theta = stateVector(2,0);
    } 

    std::vector<float> state_vector(){
      std::vector<float> vec;
      vec.push_back(x);
      vec.push_back(y);
      vec.push_back(theta);
      return vec;
    }

    MatrixXd stateToMatrix(){
      MatrixXd vec(3,1);
      vec(0,0) = x;
      vec(1,0) = y;
      vec(2,0) = theta;
      return vec;
    }

};

//Global Variables

State* prev_odom = NULL;
State* current_odom = NULL;

nav_msgs::OccupancyGrid::ConstPtr occupancyGrid;
sensor_msgs::LaserScan::ConstPtr scan;

MatrixXd noiseQk = MatrixXd::Zero(3,3);
MatrixXd Covariance_KplusOne = MatrixXd::Zero(3,3);
MatrixXd Covariance_K = MatrixXd::Zero(3,3);
MatrixXd map;
ros::Publisher publisher_scan;

int counter_steps = 0;

State* prev_state = NULL;
State* new_state = NULL;

bool Map_Active=false;

MatrixXd realZ;

double get_theta(geometry_msgs::Quaternion q){
    return tf::getYaw(q);
}

float get_velocity(float new_x, float prev_x, double delta_time){
  float distance = new_x - prev_x;
  return distance / delta_time;
}

State* f(State* prevState,State* odomState, State* currentOdom, ros::Time time_now){
  //std::cout << "delta_X->" << currentOdom->x - odomState->x << std::endl;
  //std::cout << "delta_Y->" << currentOdom->y - odomState->y << std::endl;
  //std::cout << "delta_thetas->" << currentOdom->theta - odomState->theta << std::endl;

  float x = currentOdom->x - odomState->x + prevState->x;
  float y = currentOdom->y - odomState->y + prevState->y;
  float theta = currentOdom->theta - odomState->theta + prevState->theta;
  return new State(x,y,theta, time_now);  
}

MatrixXd F(State prev_state, State new_state){
  /*std::vector<float> v; 
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
  */  
  MatrixXd m(3,3);
  m(0,0) = 1;
  m(1,0) = 0;
  m(2,0) = 0;

  m(0,1) = 0;
  m(1,1) = 1;
  m(2,1) = 0;

  m(0,2) = 0;
  m(1,2) = 0;
  m(2,2) = 1;
  

  return m;
}


MatrixXd PredictedCovariance(/*MatrixXd covarianceK,*/ State prev_state, State new_state){

  noiseQk(0,0) = 0.5;
  noiseQk(0,1) = 0;
  noiseQk(0,2) = 0;

  noiseQk(1,0) = 0;
  noiseQk(1,1) = 0.5;
  noiseQk(1,2) = 0;

  noiseQk(2,0) = 0;
  noiseQk(2,1) = 0;
  noiseQk(2,2) = 0.5;
  //printf("Nose done\n");
  MatrixXd jacobi = F(prev_state,new_state);
  //printf("F\n");  
  MatrixXd jacobiTransposed= jacobi.transpose();
  //printf("jacobiTransposed\n");
  /*std::cout << jacobi << std::endl;
  std::cout << Covariance_KplusOne << std::endl;
  std::cout << jacobiTransposed << std::endl;
  std::cout << noiseQk << std::endl;

  std::cout << "Calculated Covariance\n" << jacobi * Covariance_KplusOne * jacobiTransposed + noiseQk << std::endl;
  */
  return jacobi * Covariance_K * jacobiTransposed + noiseQk;

}



MatrixXd UpdateCovariance(MatrixXd PredictedCovariance, MatrixXd Kalman_gain, std::vector<MatrixXd> _S_KplusOne){
  MatrixXd diagonal_S_KplusOne = MatrixXd::Zero(2 * _S_KplusOne.size() ,2 * _S_KplusOne.size());

  //std::cout << "Created stacked Matrix" << std::endl;


  for(int i=0; i < _S_KplusOne.size(); i++){
    diagonal_S_KplusOne(i*2,i*2)=_S_KplusOne[i](0,0);
    diagonal_S_KplusOne(i*2,i*2 + 1)=_S_KplusOne[i](0,1);
    diagonal_S_KplusOne(i*2 + 1,i*2)=_S_KplusOne[i](1,0);
    diagonal_S_KplusOne(i*2 + 1,i*2 +1)=_S_KplusOne[i](1,1);
  }


  //std::cout << "put stuff in stacked Matrix" << std::endl;

  //std::cout << "PredictedCovariance lines ->" << PredictedCovariance.rows() << " cols->" << PredictedCovariance.cols() << std::endl;

 // std::cout << "Kalman_gain lines ->" << Kalman_gain.rows() << " cols->" << Kalman_gain.cols() << std::endl;

  //std::cout << "diagonal_S_KplusOne lines ->" << diagonal_S_KplusOne.rows() << " cols->" << diagonal_S_KplusOne.cols() << std::endl;



  return PredictedCovariance - Kalman_gain * diagonal_S_KplusOne * Kalman_gain.transpose();
}


MatrixXd h(std::vector<float> distances,float theta/*, bool thetaIsStart*/){
  float ang_incr= 0.005814; //Taken from the laserScan topic
  float scan_ang= 2.090000 * 2; //Taken from the laserScan topic
  //float ang_incr= scan_ang/3; //Taken from the laserScan topic
  
  float ang_start;
/*
  if(thetaIsStart){
    ang_start = theta;
    std::cout << "theta Is Start: " << ang_start<<std::endl;
  }
  else{
    ang_start = theta - scan_ang / 2; 
    std::cout << "theta Is not Start: " << theta<<std::endl;

  }
*/
  //MatrixXd result(2,distances.size());
  MatrixXd result = MatrixXd::Zero(2,3);
  float fi;
  int i=0;
  int j=0;
  for(; i < distances.size() ; i+= (distances.size()/ 2) - 1, j++){
    //std::cout << "J is:" << j << std::endl;
    //std::cout << "I is:" << i << std::endl;
    fi=ang_start + ang_incr * i;
    result(0,j) = distances[floor(i)];
    result(1,j) = fi;
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

MatrixXd V(MatrixXd _realZ, MatrixXd _predictedZ){
  //std::cout << "Enter V" << std::endl;
  
  MatrixXd innovation =MatrixXd::Zero(2 * _realZ.cols(), 1);

  //std::cout << "Created Innovation" << std::endl;
  for(int i = 0 ; i < _realZ.cols() ; i++){
  
    //std::cout << "Inovation Started: " << i << "Cols" << _predictedZ.cols() << std::endl;
      
    innovation(i*2,0) = _realZ(0,i) - _predictedZ(0,i);
  
    //std::cout << "Innovation middle" << std::endl;
  
    innovation(i*2 + 1,0) = _realZ(1,i) - _predictedZ(1,i);
  
    //std::cout << "Innovation ended" << std::endl;
  }
  //std::cout << "Ended V" << std::endl;

  return innovation;
}

//With E[Vij(k+1)Vij(k+1)T]



/*Receives a matrix n x n and a state and calculates the expected laser scan 
* The map data, in row-major order, starting with (0,0).  Occupancy
* probabilities are in the range [0,100].  Unknown is -1. */
std::vector<float> raycast(State s){

  std::cout << "Position: X" << (s).x << " Y:" << (s).y << " Theta:" << (s).theta << std::endl;


  int grid_x = (unsigned int)((s.x - occupancyGrid->info.origin.position.x) / occupancyGrid->info.resolution);
  int grid_y = (unsigned int)((s.y - occupancyGrid->info.origin.position.y) / occupancyGrid->info.resolution);

  std::cout << "Grid X: " << grid_x << " grid_y: " << grid_y << std::endl;

  //Parameters
  float ang_incr= 0.005814; //Taken from the laserScan topic
  float scan_ang= 2.090000 * 2; //Taken from the laserScan topic
  float resolution= occupancyGrid->info.resolution;       //TODO adjust values
  float max_range=15.600000;       //TODO adjust values        

  double number_rays= 720; 
  std::vector<float> scan;  

  float ang_start = s.theta - scan_ang / 2;
  float ang_end = s.theta + scan_ang / 2;
  

  float dist;
  float ang=ang_start;
  float ind_x;
  float ind_y;

  for(int i = 0; i < number_rays; i++){
    dist=0 - 0.125; // 0.125 is the distance between the center of the robot and the laser's positon
    ind_x = grid_x;
    ind_y = grid_y;
    
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

void transmit_laser_scan(std::vector<float> f, const sensor_msgs::LaserScan::ConstPtr& msg,ros::Publisher scan_pub)
{

  int count = 0;
  
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

}

MatrixXd prediction(ros::Time time_step){

  //std::cout << "State:" << prev_state->x << " " << prev_state->y << " " << prev_state->theta << std::endl;
  //printf("pre-f\n");


  new_state = f(prev_state,prev_odom, current_odom, time_step);

  
  Covariance_KplusOne = PredictedCovariance(*prev_state, *new_state);


  //printf("post-Covariance\n");
  std::vector<float> predictedObservation = raycast(*new_state);
  transmit_laser_scan(predictedObservation, scan, publisher_scan);

 // printf("post-Covariance\n");
  MatrixXd predictedZ = h(predictedObservation, new_state->theta);
  realZ = h(scan->ranges, current_odom->theta);
 // printf("predictedZ\n");
  

  return predictedZ;
}


State* update(State* new_state, MatrixXd kalman_Gain, MatrixXd V_matrix, std::vector<MatrixXd> SKplusOne){

  //std::cout << "Start Update" << std::endl;

  MatrixXd _update =new_state->stateToMatrix() + kalman_Gain * V_matrix;

  //std::cout << "First Middle Update" << std::endl;


  Covariance_K = UpdateCovariance( Covariance_KplusOne, kalman_Gain, SKplusOne);

  //std::cout << "Middle Update" << std::endl;

  State* updatedState= new State(_update);

  //std::cout << "End Update" << std::endl;

  return updatedState;
}



//Kalman Gain
MatrixXd KkplusOne(MatrixXd CovarianceKplusOne,std::vector<MatrixXd> H,std::vector<MatrixXd> S_KplusOne){
  
  //std::cout << "KkplusOne Inicio" << std::endl;

  int H_size = H.size();
  
  
  MatrixXd stacked_H = MatrixXd::Zero(H_size * 2, 3);

  MatrixXd diagonal_S_KplusOne = MatrixXd::Zero(H_size * 2 , H_size * 2);

  MatrixXd real_kalman_Gain = MatrixXd::Zero(3,H_size*2);


  for(int i=0; i < H_size; i++){
    stacked_H(i*2, 0)=H[i](0,0);
    stacked_H(i*2, 1)=H[i](0,1);
    stacked_H(i*2 + 1, 0)=H[i](1,0);
    stacked_H(i*2 + 1, 1)=H[i](1,1);
  }


 // std::cout << "S: \n" << diagonal_S_KplusOne << std::endl;

  for(int i=0; i < H_size; i++){
    diagonal_S_KplusOne(i*2,i*2)=S_KplusOne[i](0,0);
    diagonal_S_KplusOne(i*2,i*2 + 1)=S_KplusOne[i](0,1);
    diagonal_S_KplusOne(i*2 + 1,i*2)=S_KplusOne[i](1,0);
    diagonal_S_KplusOne(i*2 + 1,i*2 +1)=S_KplusOne[i](1,1);
  }

 // std::cout << "Begin For KkplusOne" << std::endl;
  
 // std::cout << "S: \n" << diagonal_S_KplusOne << std::endl;
 // std::cout << "Inverse S: \n" << diagonal_S_KplusOne.inverse() << std::endl;

  real_kalman_Gain = CovarianceKplusOne * stacked_H.transpose() * diagonal_S_KplusOne.inverse();
  
 // std::cout << "KkplusOne end" << std::endl;

  return real_kalman_Gain;
}


std::vector<MatrixXd> S_KplusOne(std::vector<MatrixXd> H, MatrixXd CovarianceKplusOne ){
  
  std::vector<MatrixXd> _S_KplusOne;
  MatrixXd Rk(2,2);
  Rk << 0.5, 0,
        0,    0.5;

  for(int i=0; i < H.size(); i++){
    //std::cout << Hi << std::endl;
    _S_KplusOne.push_back(H[i] * CovarianceKplusOne * H[i].transpose() + Rk);  
  }
  

  return _S_KplusOne;
}


std::vector<int> matching(std::vector<MatrixXd> S_KplusOne, MatrixXd _V){
  //TODO Change GAMA
  int Gama = 50;
  std::vector<int> index_to_include;
  MatrixXd vij(2,1);
  for(int i=0; i < _V.rows() / 2; i++){
    vij(0,0) = _V(i,0);
    vij(1,0) = _V(i+1,0);
    MatrixXd val = vij.transpose() * S_KplusOne[i] * vij; 
    if(val(0,0) <= Gama){
      index_to_include.push_back(i);
    }
  }
  return index_to_include;
}





void ekf_step(ros::Time time_step){
  //init in first step
  if(counter_steps == 0)
  {
   // printf("First Step\n");
    float x = current_odom->x;
    float y = current_odom->y;
    float theta = current_odom->theta;
    prev_state = new State(x,y,theta, time_step);
    prev_odom = new State(x,y,theta,time_step);
    Covariance_K << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;
    //printf("First Step\n"); 

  }
  else
  {
    //std::cout <<"Covariance_KplusOne: " << Covariance_KplusOne << std::endl;

    //std::cout <<"Covariance_K: " << Covariance_K << std::endl;

    //Prediction Step
   // printf("pre-prediction\n");
    MatrixXd predictedZ  = prediction(time_step);
    
   // std::cout <<"predictedZ: " << predictedZ << std::endl;

    //printf("pos-prediction\n");
    std::vector<MatrixXd> _H = H(predictedZ,*new_state);

    /*for(int i = 0; i < _H.size(); i++){
      std::cout <<"_H[" << i << "]: " << _H[i] << std::endl;
    }*/

    //printf("H\n");  
    std::vector<MatrixXd> SKplusOne = S_KplusOne( _H, Covariance_KplusOne /* RKplusOne*/);

  /*  for(int i = 0; i < SKplusOne.size(); i++){
      std::cout <<"SKplusOne[" << i << "]: " << SKplusOne[i] << std::endl;
    }*/
   // printf("S_KplusOne\n");
    //Matching Step
   // std::cout << "realZ:" << realZ << std::endl;
   // std::cout << "predictedZ:" << predictedZ << std::endl;

    MatrixXd _V = V(realZ, predictedZ); //Who you gonna call? MATH DUDE! How do we accept the value??
    

    std::vector<int> index_to_include = matching(SKplusOne, _V);

    std::vector<MatrixXd> _H_copy;
    std::vector<MatrixXd> SKplusOne_copy;
    MatrixXd _V_copy(index_to_include.size()*2, 1);

    for(int i=0; i < index_to_include.size(); i++){
      int j = index_to_include[i];
      _H_copy.push_back(_H[j]);
      SKplusOne_copy.push_back(SKplusOne[j]);
      _V_copy(i*2,0) = _V(j*2,0);
      _V_copy(i*2 +1,0) = _V(j*2 +1,0);
    }

    _H = _H_copy;
    SKplusOne = SKplusOne_copy;
    _V = _V_copy;

   // std::cout <<"_V: " << _V << std::endl;


   // printf("V\n");

  //  std::cout <<"Kalman_gain: " << KkplusOne(Covariance_KplusOne, _H, SKplusOne) << std::endl;


    //Update Step
    prev_state=update(new_state,KkplusOne(Covariance_KplusOne, _H, SKplusOne), _V, SKplusOne);


   // printf("update\n");
    prev_odom=current_odom;
  //  printf("prev_odom\n");
  }
}



MatrixXd occupancyGridToMatrix(){
  MatrixXd m = MatrixXd::Zero(occupancyGrid->info.height, occupancyGrid->info.width);

  for(int line=0; line < occupancyGrid->info.height; line++){
    for(int column=0; column < occupancyGrid->info.width; column++){
      m(line,column) = occupancyGrid->data[column + line * occupancyGrid->info.width];
    }    
  }
  
  //std::cout << "Map matrix size ---->" << m.rows() << "x" << m.cols() << std::endl;

  return m;
}

void odom_receiver(const nav_msgs::Odometry::ConstPtr& msg)
{
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  double theta = get_theta(msg->pose.pose.orientation);
  current_odom = new State(x,y,theta);
}

void map_receiver(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  
  occupancyGrid = msg;
  
  map = occupancyGridToMatrix();

  Map_Active=true;
}





void transmit_state(State* s, ros::Publisher odom_pub)
{
  
  //generate some fake data for our laser scan
  ros::Time state_time = ros::Time::now();
  
  //printf("quaternion_calc\n");
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(s->theta);
  
  //printf("preODOM\n");
  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  //Header
  odom.header.stamp = state_time;
  odom.header.seq = counter_steps;
  odom.header.frame_id = "odom";



  //printf("preState\n");
  //set the position
  odom.pose.pose.position.x = s->x;
  odom.pose.pose.position.y = s->y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;


  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = 0;

  //publish the message
  odom_pub.publish(odom);
  //printf("postPublish\n");
  
}


void scan_receiver(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan=msg;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ekf");   

  ros::NodeHandle n;

  ros::Subscriber sub_odom = n.subscribe("odom", 1000, odom_receiver);
  ros::Subscriber sub_scan = n.subscribe("base_scan", 1000, scan_receiver);
  ros::Subscriber sub_map = n.subscribe("/map_from_map_server", 1000, map_receiver);    //Has to subscribe to our moded_map_server topic in order to avoid rewriting the map

  ros::Publisher pub_new_estimates = n.advertise<nav_msgs::Odometry>("EKF_New_State", 1000);
  publisher_scan = n.advertise<sensor_msgs::LaserScan>("bla_scan", 50);


  ros::Rate loop_rate(10);   

  while (ros::ok())
  {
    
    if(Map_Active){
      //printf("ekf_step\n");
      ekf_step(ros::Time::now());
      
      //printf("transmit\n");
      transmit_state(prev_state, pub_new_estimates);

//      printf("end\n");
      counter_steps++;
    }
    
    ros::spinOnce();

    loop_rate.sleep();
  

  }


  ros::spin();

  return 0;
}
