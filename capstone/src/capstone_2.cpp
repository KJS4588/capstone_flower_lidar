#include "capstone/capstone.h"

void Capstone::initSetup() {
	yaw_ = 0;
	left_temp_ = 0.0;
	right_temp_ = 0.0;
	gap_ = 0.0;
	pre_gap_ = 0.0;
	angle_ = 0.0; pre_angle_ = 0.0;

	scan_sub_ = nh_.subscribe("/scan", 10, &Capstone::scanCallback, this);
	cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd", 10);
}

void Capstone::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan){
	int count = 0;
	for (int i=90;i<=270;i++) { // channel per 5 degree data 
		if (i%5==0) {
			scan_data_[count] = scan->ranges[i]>8||scan->ranges[i]<1 ? THE : scan->ranges[i]*scan->ranges[i];
			count++;
		}
	} 

	for (int i=0;i<18;i++) {
		right_temp_ += scan_data_[i];
	}	

	for (int i=18;i<36;i++) {
		left_temp_ += scan_data_[i];
	}
	
	gap_ = right_temp_ - left_temp_;
	
	right_temp_ = 0; 
	left_temp_ = 0;

	angle_ = gap_ * 2 + 1500;

	if (angle_ < 1300) angle_ = 1300;
	if (angle_ > 1700) angle_ = 1700;
	
	cmd_.angular.z = angle_;
	cmd_.linear.x = 95;
	
	pre_angle_ = angle_;
	pre_gap_ = gap_;

	cmd_pub_.publish(cmd_);
	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Capstone_node2");
	Capstone ct;
	ros::spin();
	return 0;
}

