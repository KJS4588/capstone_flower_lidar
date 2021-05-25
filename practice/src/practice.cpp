#include "ros/ros.h"
#include "iostream"

#include "geometry_msgs/Twist.h"
using namespace std;

ros::Publisher cmd_pub_;

int main(int argc, char** argv){
	ros::init(argc, argv, "practice_node");
	ros::NodeHandle n;

	ros::Rate r(10);
	cmd_pub_ = n.advertise<geometry_msgs::Twist>("cmd", 10);

	geometry_msgs::Twist cmd;
	
	n.setParam("/steer", 0);
	n.setParam("/vel", 0);
	while (ros::ok()){
		int vel = 0;
		int steer = 0;
		n.getParam("/steer", steer);
		n.getParam("/vel", vel);
		
		cmd.angular.z = steer;
		cmd.linear.x = vel;

		cmd_pub_.publish(cmd);
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
