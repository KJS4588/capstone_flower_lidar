#include "ros/ros.h"
#include "obstacle_detector/Obstacles.h"
#include "obstacle_detector/SegmentObstacle.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

#include "vector"
#include "cmath"

#define WHEEL_BASE 0.25 // cm
#define LD         1    // m

using namespace std;

class Capstone{
private:
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
	ros::Publisher cmd_pub_, marker_pub_;

	vector<geometry_msgs::Point> obs_;

public:
	Capstone() { initSetup(); }
	~Capstone() {}

	void initSetup();
	void scanCallback(const obstacle_detector::Obstacles obs);
	
	geometry_msgs::Point checkObstacle(const obstacle_detector::Obstacles obs);
	
	double calcDistance(geometry_msgs::Point point) { return sqrt(point.x*point.x + point.y*point.y); }
	double calcAnglue(geometry_msgs::Point point);

};
