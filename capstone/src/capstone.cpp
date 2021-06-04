#include "capstone/capstone.h"

void Capstone::initSetup() {
//	scan_sub_ = nh_.subscribe("/raw_obstacles", 10, &Capstone::scanCallback, this);
//	imu_sub_ = nh_.subscribe("/imu/data", 10, &Capstone::imuCallback, this);
	coef_sub_ = nh_.subscribe("/coef", 10, &Capstone::wayCallback, this);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint", 10);
	cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd", 10);
	state_pub_ = nh_.advertise<std_msgs::Bool>("/state", 10);
	
	alpha_ = 0.0;
	yaw_ = 0.0;
	des_angle_ = 0.0;
	intersection_ = false;
}

void Capstone::wayCallback(const std_msgs::Float32MultiArray::ConstPtr &coef) {
	double way_coef[3] = {0};

	for (int i=0;i<3;i++) {
		way_coef[i] = coef->data[i];
	}	
	goal_point_ = makeGoalpoint(way_coef);
	alpha_ = calcAngle(goal_point_);
	angle_ = calcSteeringAngle(abs(alpha_)) * 180 / M_PI;

	if (alpha_ > 0) angle_ = -angle_;
	
	cmd_.linear.x = 1550;
	cmd_.angular.z = angle_ * 150/20 + 1500;
	if (cmd_.angular.z < 1350) cmd_.angular.z = 1350;
	if (cmd_.angular.z > 1750) cmd_.angular.z = 1750;
	if (cmd_.angular.z > 1580) intersection_ = true;
	
	state_.data = intersection_;

	state_pub_.publish(state_);
	cout << endl << "angle : " << angle_ << endl;
	cout << "cmd   : " << cmd_.angular.z << endl << endl;	
	cmd_pub_.publish(cmd_);
}

/*
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
}*/



/*
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
	cmd_.linear.x = 95;

	cout << 90 - calcAngle(point) << endl;
	cmd_.angular.z = (90 - calcAngle(point)) * 1500/90;
	if (cmd_.angular.z >=1700) cmd_.angular.z = 1700;
	if (cmd_.angular.z <=1300)  cmd_.angular.z = 1300;
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
}*/


double Capstone::calcSteeringAngle(double alpha) {
	return atan2(2*WHEEL_BASE*sin(alpha), LD);
}
double Capstone::calcAngle(geometry_msgs::Point point) {
	return atan2(point.y, point.x); 
}

geometry_msgs::Point Capstone::makeGoalpoint(double coef[ORDER+1]) {
	geometry_msgs::Point p;
	p.y = coef[3]*LD*LD + coef[2]*LD*LD + coef[1]*LD + coef[0];
	p.x = LD;
	p.z = 0;

	return p;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "capstone_node");
	Capstone ct;
	ct.initSetup();
	ros::spin();
	return 0;
}
