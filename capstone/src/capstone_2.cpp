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
	double sum = 0.0;

	for (int i=90;i<=270;i++) { // channel per 5 degree data 
		if (i%5==0) {
			scan_data_[count] = scan->ranges[i]>8||scan->ranges[i]<1 ? THE : scan->ranges[i];
			count++;
		}
	} 

	for (int i=0;i<36;i++) {
		if (scan_data_[i] > 8) idx_.push_back(i);
	}

	if (idx_.size() != 0) {
		left_temp_ = idx_.at(idx_.size()-1);
		right_temp_ = idx_.at(0);

		left_temp_ = 18 - left_temp_;
		right_temp_ = 18 - right_temp_;
		
		if (abs(left_temp_) == abs(right_temp_)) angle_ = 1500;
		else {
			sum = abs(left_temp_) > abs(right_temp_) ? (left_temp_) * 5 : (right_temp_) * 5;
		}

		angle_ = 200/13*sum + 1500;
	} else {
	}

	if (angle_ < 1350) angle_ = 1350;
	if (angle_ > 1650) angle_ = 1650;


	cmd_.angular.z = angle_;
	cmd_.linear.x = 95;
	
	cmd_pub_.publish(cmd_);
	idx_.clear();
	/*
	cout << "sum       : " << sum << endl;
	cout << "angle     : " << angle_ << endl;
	cout << "angular_z : " << cmd_.angular.z << endl;
*/
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Capstone_node2");
	Capstone ct;
	ct.initSetup();
	ros::spin();
	return 0;
}

