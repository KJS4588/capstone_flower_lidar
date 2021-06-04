#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"
#include "std_msgs/Bool.h"

using namespace std;

class Converter {
	public:
		void initSetup();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
		void stateCallback(const std_msgs::Bool::ConstPtr &msg);
	private:
		ros::NodeHandle nh_;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener tfListener_;

		ros::Publisher point_pub_, pub_;
		ros::Subscriber scan_sub_, state_sub_;
		
		bool is_intersection_;

        float scan_data_[36] = {0};
};

void Converter::initSetup() {
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &Converter::scanCallback, this);
	state_sub_ = nh_.subscribe<std_msgs::Bool>("/state", 100, &Converter::stateCallback, this);
	pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered",100);
	point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 100, false);
	tfListener_.setExtrapolationLimit(ros::Duration(0.1));
	is_intersection_ = false;
}


void Converter::stateCallback(const std_msgs::Bool::ConstPtr& msg) {
	is_intersection_ = msg->data;

}
void Converter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    
	int count = 0;
	sensor_msgs::LaserScan scan;
	scan.header = msg->header;
	scan.angle_min = msg->angle_min;
	scan.angle_max = msg->angle_max;
	scan.angle_increment = msg->angle_increment;
	scan.time_increment = msg->time_increment;
	scan.scan_time = msg->scan_time;
	scan.range_min = msg->range_min;
	scan.range_max = msg->range_max;
	if (!is_intersection_) {
		for (int i=0;i<75;i++){
			scan.ranges.push_back(numeric_limits<float>::infinity());
			scan.intensities.push_back(0);
		}
		for (int i=75;i<=285;i++){
			float msg_data = msg->ranges[i]>8||msg->ranges[i]<0 ? numeric_limits<float>::infinity() : msg->ranges[i];

			scan.ranges.push_back(msg_data);
			scan.intensities.push_back(msg->intensities[i]);
			if (i%5==0) {
				scan_data_[count] = msg_data>8||msg_data<0 ? numeric_limits<float>::infinity() : msg_data;
				count++;
			}
		}

		for (int i=286;i<360;i++){
			scan.ranges.push_back(numeric_limits<float>::infinity());
			scan.intensities.push_back(0);
		}
		cout << endl << "No intersection" << endl;
	 } else {
		 for (int i=0;i<75;i++){
			scan.ranges.push_back(numeric_limits<float>::infinity());
			scan.intensities.push_back(0);
		}
		for (int i=75;i<=285;i++){
			float msg_data = msg->ranges[i]>2.4||msg->ranges[i]<0 ? numeric_limits<float>::infinity() : msg->ranges[i];

			scan.ranges.push_back(msg_data);
			scan.intensities.push_back(msg->intensities[i]);
			if (i%5==0) {
				scan_data_[count] = msg_data>8||msg_data<0 ? numeric_limits<float>::infinity() : msg_data;
				count++;
			}
		}

		for (int i=286;i<360;i++){
			scan.ranges.push_back(numeric_limits<float>::infinity());
			scan.intensities.push_back(0);
		}
	 
		cout << endl << "intersection" << endl;
	 }

	pub_.publish(scan);

    sensor_msgs::PointCloud2 cloud;
	projector_.transformLaserScanToPointCloud("/laser", scan, cloud, tfListener_);
	point_pub_.publish(cloud);

}



int main(int argc, char **argv) {
	ros::init(argc, argv, "converter");
	Converter Cv;
	Cv.initSetup();
	ros::spin();
	
	return 0;
}
