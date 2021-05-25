#include "capstone/capstone.h"

void Capstone::initSetup() {
	scan_sub_ = nh_.subscribe("/raw_obstacles", 10, &Capstone::scanCallback, this);
	imu_sub_ = nh_.subscribe("/imu/data", 10, &Capstone::imuCallback, this);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint", 10);
	cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd", 10);
}

void Capstone::imuCallback(const sensor_msgs::ImuConstPtr &data){
	double roll, pitch, yaw = 0;
	
	tf::Quaternion q(
			data->orientation.x,
			data->orientation.y,
			data->orientation.z,
			data->orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	yaw_ =  yaw * 180 / M_PI;
	
	cout << yaw_ << endl;
}

void Capstone::scanCallback(const obstacle_detector::Obstacles obs){
	geometry_msgs::Point point = checkObstacle(obs);

	visualization_msgs::Marker points;

	points.header.frame_id = "laser";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1; 
	points.scale.y = 0.1;
	points.color.a = 1.0;

	points.color.b = 1.0f;

	geometry_msgs::Point p;

	p.x = point.x;
	p.y = point.y;
	p.z = point.z;
	points.points.push_back(p);

	marker_pub_.publish(points);
	cmd_.linear.x = 100;
	cmd_.angular.z = 90 - calcAngle(point);
	cmd_pub_.publish(cmd_);
}

geometry_msgs::Point Capstone::checkObstacle(const obstacle_detector::Obstacles obs){
	geometry_msgs::Point point;
	for (auto segment : obs.segments) {
		
		point.x += (segment.first_point.x + segment.last_point.x)/2;
		point.y += (segment.first_point.y + segment.last_point.y)/2;
		point.z = 0;
	}

	point.x = point.x/obs.segments.size();
	point.y = point.y/obs.segments.size();
	
	return point;
}

double Capstone::calcAngle(geometry_msgs::Point point) {
	return atan2(point.y, point.x) * 180 / M_PI; 
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "capstone_node");
	Capstone ct;
	ros::spin();
	return 0;
}
