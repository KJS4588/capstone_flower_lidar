#include "ros/ros.h"
#include "obstacle_detector/Obstacles.h"
#include "obstacle_detector/SegmentObstacle.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"

#include "vector"
#include "cmath"

#define WHEEL_BASE 0.25 // cm
#define LD         1    // m
#define _USE_MATH_DEFINES

using namespace std;

class Capstone{
private:
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_, imu_sub_;
	ros::Publisher cmd_pub_, marker_pub_;

	vector<geometry_msgs::Point> obs_;
	
	double yaw_;
	geometry_msgs::Twist cmd_;
public:
	Capstone() { initSetup(); }
	~Capstone() {}

	void initSetup();
	void scanCallback(const obstacle_detector::Obstacles obs);
	void imuCallback(const sensor_msgs::ImuConstPtr &data);
	
	geometry_msgs::Point checkObstacle(const obstacle_detector::Obstacles obs);
	
	double calcDistance(geometry_msgs::Point point) { return sqrt(point.x*point.x + point.y*point.y); }
	double calcAngle(geometry_msgs::Point point);

};
