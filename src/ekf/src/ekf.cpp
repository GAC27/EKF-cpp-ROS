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
#include <limits>
#include <laser_assembler/AssembleScans.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/normal_3d.h>

using namespace laser_assembler;
using Eigen::MatrixXd;
using Eigen::MatrixXf;


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

pcl::PointCloud<pcl::PointXYZ>::Ptr realPCLCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr predictedPCLCloud;

MatrixXd noiseQk = MatrixXd::Zero(3,3);
MatrixXd Covariance_KplusOne = MatrixXd::Zero(3,3);
MatrixXd Covariance_K = MatrixXd::Zero(3,3);
MatrixXd map;
ros::Publisher publisher_scan;
ros::Publisher real_point_cloud_publisher_;
ros::Publisher predicted_point_cloud_publisher_;
ros::Publisher location_undertainty;

laser_geometry::LaserProjection projector;
tf::TransformListener* listener;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>  icp_matching(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

int x_covariance;
int y_covariance;


//~ DEBUG
pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertFromPointCloud2ToPCL(sensor_msgs::PointCloud2 msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
	//do stuff with temp_cloud here
	return temp_cloud;
}
//~ DEBUG

int counter_steps = 0;

State* prev_state = NULL;
State* new_state = NULL;

bool Map_Active=false;
bool Odom_Active=false;
bool Scan_Active=false;

MatrixXd realZ;

double get_theta(geometry_msgs::Quaternion q){
    return tf::getYaw(q);
}

float get_velocity(float new_x, float prev_x, double delta_time){
  float distance = new_x - prev_x;
  return distance / delta_time;
}


void drawCovariance(const Eigen::Matrix2f& covMatrix)
{
    visualization_msgs::Marker tempMarker;
    tempMarker.pose.position.x = 0;
    tempMarker.pose.position.y = 0;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

    const Eigen::Vector2f& eigValues (eig.eigenvalues());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

    float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));


    tempMarker.type = visualization_msgs::Marker::SPHERE;

    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);

    tempMarker.scale.x = 2.4477*lengthMajor;
    x_covariance = tempMarker.scale.x;
    tempMarker.scale.y = 2.4477*lengthMinor;
    y_covariance = tempMarker.scale.y;
    tempMarker.scale.z = 0.001;

    tempMarker.color.a = 1.0;
    tempMarker.color.r = 1.0;
	
	tempMarker.pose.position.x = new_state->x;
	tempMarker.pose.position.y = new_state->y;
	tempMarker.pose.position.z = 0.5;
	
    tempMarker.pose.orientation.w = cos(angle*0.5);
    tempMarker.pose.orientation.z = sin(angle*0.5);

    tempMarker.header.frame_id="odom";
    tempMarker.id = 0;

    location_undertainty.publish(tempMarker);
}





State* f(State* prevState,State* odomState, State* currentOdom, ros::Time time_now){
	float x = currentOdom->x - odomState->x + prevState->x;
	float y = currentOdom->y - odomState->y + prevState->y;
	float theta = currentOdom->theta - odomState->theta + prevState->theta;
	std::cout << "Difference of X: " << currentOdom->x - odomState->x << std::endl;
	std::cout << "Difference of Y: " << currentOdom->y - odomState->y << std::endl; 
	std::cout << "Difference of Theta: " << currentOdom->theta - odomState->theta << std::endl;
	return new State(x,y,theta, time_now);  
}

MatrixXd F(State prev_state, State new_state){
  MatrixXd m(3,3);
  m << 1, 0, -(new_state.y-prev_state.y),
	   0, 1, -(new_state.x-prev_state.x),
	   0, 0, 1;
  return m;
}


MatrixXd PredictedCovariance(State prev_state, State new_state, State* currentOdom, State* odomState){
	
	
	double noiseOdomX = 0.001 * std::abs(currentOdom->x - odomState->x);
	double noiseOdomY = 0.001 * std::abs(currentOdom->y - odomState->y);
	double noiseTheta = 0.0011 * std::abs(currentOdom->theta - odomState->theta);
	noiseQk << 0.0005 + noiseOdomX, 0, 0,
			   0, 0.0005 + noiseOdomY, 0,
			   0, 0, 0.0005 + noiseTheta;
			   
	MatrixXd jacobi = F(prev_state,new_state);
	MatrixXd jacobiTransposed= jacobi.transpose();

	return jacobi * Covariance_K * jacobiTransposed + noiseQk;
}



MatrixXd UpdateCovariance(MatrixXd PredictedCovariance, MatrixXd Kalman_gain, MatrixXd _S_KplusOne){
	MatrixXd identity(3,3);
	identity <<  1, 0, 0,
				 0, 1, 0,
				 0, 0, 1;
	std::cout << "Update Covarince" << std::endl;
	std::cout << "Kalman Gain" << Kalman_gain << std::endl;
	std::cout << "PredictedCovariance" <<  PredictedCovariance << std::endl;
	std::cout << "UpdateCovariance Before returning" <<  (identity - Kalman_gain) * PredictedCovariance << std::endl;
	std::cout << "PredictedCovariance Ended" << std::endl;
	return (identity - Kalman_gain) * PredictedCovariance;
}



MatrixXd H(){
	MatrixXd _H(3,3);
	_H <<  1, 0, 0,
		   0, 1, 0,
		   0, 0, 1;
	return _H;
}

MatrixXd V(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp){
	MatrixXf icpTransform = icp.getFinalTransformation();

	//~ std::cout << "has converged:" << icp.hasConverged() << " score: " << 
	//~ icp.getFitnessScore() << std::endl;
	
	MatrixXd Vvector(3,1);
	Vvector(0,0) = (double) icpTransform(0,3);
	
	Vvector(1,0) = (double) icpTransform(1,3);
	
	Vvector(2,0) = atan2((double) icpTransform(0,1), (double) icpTransform(0,0));
	
	return Vvector;
}

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
	float resolution= occupancyGrid->info.resolution;
	float max_range=5.600000;        

	double number_rays= 720; 
	std::vector<float> scan;  

	float ang_start = s.theta - scan_ang / 2;
	float ang_end = s.theta + scan_ang / 2;


	float dist;
	float ang=ang_start;
	float ind_x;
	float ind_y;

	for(int i = 0; i < number_rays; i++, ang += ang_incr ){
		dist=0;
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
		
	} 

	return scan;
}

void transmit_laser_scan(std::vector<float> f, const sensor_msgs::LaserScan::ConstPtr& msg,ros::Publisher scan_pub)
{  
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
  
  sensor_msgs::PointCloud2 cloud;
  projector.transformLaserScanToPointCloud("base_link",scan, cloud,*listener);
  predictedPCLCloud = ConvertFromPointCloud2ToPCL(cloud);

  // Do something with cloud.
  predicted_point_cloud_publisher_.publish(cloud);
  

}

void prediction(ros::Time time_step){

	new_state = f(prev_state,prev_odom, current_odom, time_step);

	Covariance_KplusOne = PredictedCovariance(*prev_state, *new_state, current_odom, prev_odom);
	
	std::vector<float> predictedObservation = raycast(*new_state);
	transmit_laser_scan(predictedObservation, scan, publisher_scan);
	
}


State* update(State* new_state, MatrixXd kalman_Gain, MatrixXd V_matrix, MatrixXd SKplusOne){
	
	MatrixXd _update = new_state->stateToMatrix() + kalman_Gain * V_matrix;
	
	
	//if( std::abs(_update(0,0) - new_state->x) < x_covariance && std::abs(_update(1,0) - new_state->y) < y_covariance){
		Covariance_K = UpdateCovariance(Covariance_KplusOne, kalman_Gain, SKplusOne);

		State* updatedState= new State(_update);

		return updatedState;
	/*}
	else
	{
		Covariance_K=Covariance_KplusOne;
		return pre_state;
	}*/
	
}



//Kalman Gain
MatrixXd KkplusOne(MatrixXd CovarianceKplusOne,MatrixXd H, MatrixXd S_KplusOne){
	MatrixXd Rk(3,3);
	Rk << 0.1, 0, 0,
		    0, 0.1, 0,
		    0, 0, 0.1;
	
	return CovarianceKplusOne * (CovarianceKplusOne + Rk).inverse();
}


MatrixXd S_KplusOne(MatrixXd H, MatrixXd CovarianceKplusOne ){
  
	MatrixXd _S_KplusOne;

	MatrixXd Rk(3,3);
	Rk << 0.1, 0, 0,
		    0, 0.1, 0,
		    0, 0, 0.1;

	_S_KplusOne = H * CovarianceKplusOne * H.transpose() + Rk;


	return _S_KplusOne;
}


std::vector<int> matching(std::vector<MatrixXd> S_KplusOne, MatrixXd _V){
	//TODO Change GAMMA MathDude Stuff
	float Gama = 0.08;
	std::vector<int> index_to_include;
	MatrixXd vij(2,1);
	MatrixXd val(1,1);
	
	for(int i=0; i < _V.rows() / 2; i++){		  
		vij(0,0) = _V(i,0);
		vij(1,0) = _V(i+1,0);
		val = vij.transpose() * S_KplusOne[i].inverse() * vij; 
		if(val(0,0) <= Gama){
			index_to_include.push_back(i);
		}
	}
	
	return index_to_include;
}



void ekf_step(ros::Time time_step){
	//init in first step
	std::cout << "Start Ekf Step" << std::endl;
	if(counter_steps == 0)
	{
		float x = current_odom->x;
		float y = current_odom->y;
		float theta = current_odom->theta;
		prev_state = new State(x,y,theta, time_step);
		prev_odom = new State(x,y,theta,time_step);
		Covariance_K << 0.06, 0, 0,
						0, 0.06, 0,
						0, 0, 0.06;
	}
	else
	{
		std::cout << "Second start Ekf Step" << std::endl;
		//Prediction Step
		prediction(time_step);

		MatrixXd _H = H();

		MatrixXd SKplusOne = S_KplusOne( _H, Covariance_KplusOne);
		
		//Matching Step
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>  icpAligned= icp_matching(predictedPCLCloud,realPCLCloud);
		boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
		pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
		corr_est.setInputSource (predictedPCLCloud);
		corr_est.setInputTarget (realPCLCloud);
		corr_est.determineCorrespondences (*correspondences);
		if(icpAligned.hasConverged() && correspondences->size() >= 400){		//Match
			MatrixXd _V = V(icpAligned); 
			std::cout << "V: " << _V << std::endl;
			//Update Step
			prev_state=update(new_state,KkplusOne(Covariance_KplusOne, _H, SKplusOne), _V, SKplusOne);
		}		
		else{ 	//No Match
			//Update Step
			prev_state=new_state;
			Covariance_K=Covariance_KplusOne;
		}
		
		Eigen::Matrix2f covMatrix;
		covMatrix(0,0)=Covariance_K(1,1);
        covMatrix(0,1)=Covariance_K(1,2);
        covMatrix(1,0)=Covariance_K(2,1);
        covMatrix(1,1)=Covariance_K(2,2);
		drawCovariance(covMatrix);
		
		std::cout<< "Kalman Gain: " << KkplusOne(Covariance_KplusOne, _H, SKplusOne) << std::endl;
		std::cout<< "Cov(K+1|K)=\n" << Covariance_KplusOne << "\nCov(K+1|K+1)=\n" << Covariance_K << std::endl;
		std::cout<< (correspondences->size()) << std::endl;
		prev_odom=current_odom;
		
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
  	Odom_Active=true;
}

void map_receiver(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  
  occupancyGrid = msg;
  
  map = occupancyGridToMatrix();

  Map_Active=true;
}

///Receives a two PCL point clouds. cloud_in is the Predicted Observation and cloud_out is the Real Observation
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_matching(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);	//Predicted Observation
	icp.setInputTarget(cloud_out); 	//Real Observation
	icp.setMaxCorrespondenceDistance(0.01);//
	icp.setMaximumIterations(700); // First Criteria
	icp.setTransformationEpsilon(1e-7); //Second Criteria
	//icp.setEuclideanFitnessEpsilon(0.0001); // Third Criteria
	icp.setRANSACIterations (1000);
	icp.setRANSACOutlierRejectionThreshold(0.1);
	
	
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	real_point_cloud_publisher_.publish(Final);
	
	return icp;
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

void bla(){
	sensor_msgs::LaserScan laser_scan;
	laser_scan.header = scan->header;
	laser_scan.angle_min = scan->angle_min;
	laser_scan.angle_max = scan->angle_max;
	laser_scan.angle_increment = scan->angle_increment;
	laser_scan.time_increment = scan->time_increment;
	laser_scan.range_min = scan->range_min;
	laser_scan.range_max = scan->range_max;
	laser_scan.ranges = scan->ranges;
	laser_scan.intensities = scan->intensities;

	ros::Time time_n = ros::Time::now();
	sensor_msgs::PointCloud2 cloud;
	laser_scan.header.stamp = time_n;
	projector.transformLaserScanToPointCloud("base_link",laser_scan, cloud,*listener);
	realPCLCloud = ConvertFromPointCloud2ToPCL(cloud);
	// Do something with cloud.
	//real_point_cloud_publisher_.publish(cloud);
}


void scan_receiver(const sensor_msgs::LaserScan::ConstPtr& msg){
	scan=msg;
	bla();
	Scan_Active=true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ekf");   

	ros::NodeHandle n;

	listener= new tf::TransformListener();
	
	ros::Subscriber sub_odom = n.subscribe("odom", 1000, odom_receiver);
	ros::Subscriber sub_scan = n.subscribe("base_scan", 1000, scan_receiver);
	ros::Subscriber sub_map = n.subscribe("/map_from_map_server", 1000, map_receiver);    //Has to subscribe to our moded_map_server topic in order to avoid rewriting the map

	ros::Publisher pub_new_estimates = n.advertise<nav_msgs::Odometry>("EKF_New_State", 1000);
	publisher_scan = n.advertise<sensor_msgs::LaserScan>("bla_scan", 50);
	predicted_point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud2> ("predicted_scan_cloud", 100, false);
	real_point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud2> ("base_scan_cloud", 100, false);
	location_undertainty = n.advertise<visualization_msgs::Marker>("/location_undertainty",1);
	ros::Rate loop_rate(10);   
	
	while (ros::ok())
	{

		if(Map_Active && Scan_Active && Odom_Active){
			
		  ekf_step(ros::Time::now());
		  
		  transmit_state(prev_state, pub_new_estimates);

		  counter_steps++;
		}

		ros::spinOnce();

		loop_rate.sleep();


	}


	ros::spin();

	return 0;
}
