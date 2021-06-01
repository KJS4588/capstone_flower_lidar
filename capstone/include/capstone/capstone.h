#include "ros/ros.h"
#include "obstacle_detector/Obstacles.h"
#include "obstacle_detector/SegmentObstacle.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/tf.h"

#include "vector"
#include "cmath"

#define WHEEL_BASE 0.25 // cm
#define LD         2    // m
#define WIDTH      2.1  // m
#define THE        numeric_limits<float>::infinity()
#define _USE_MATH_DEFINES

using namespace std;

class Capstone{
private:
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_, imu_sub_, coef_sub_;
	ros::Publisher cmd_pub_, marker_pub_;

	vector<int> idx_;
	float scan_data_[36] = {0};	
	
	double alpha_, angle_, des_angle_; //degree
	double gap_, pre_gap_;               //between left & right
	double yaw_, left_temp_, right_temp_;
		
	int count_;

	geometry_msgs::Point goal_point_;
	geometry_msgs::Twist cmd_;
public:

	void initSetup();
	void wayCallback(const std_msgs::Float32MultiArray::ConstPtr &coef);	
	
	//	void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
	//void scanCallback(const obstacle_detector::Obstacles obs);
	//	void imuCallback(const sensor_msgs::ImuConstPtr &data);
	
	//geometry_msgs::Point checkObstacle(const obstacle_detector::Obstacles obs);
	
	double calcDistance(geometry_msgs::Point point) { return sqrt(point.x*point.x + point.y*point.y); }
	double calcAngle(geometry_msgs::Point point);
	double calcSteeringAngle(double alpha);

	geometry_msgs::Point makeGoalpoint(double coef[3]);
};
