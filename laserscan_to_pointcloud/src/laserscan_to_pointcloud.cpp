#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"

class Converter {
	public:
		void initSetup();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
	
	private:
		ros::NodeHandle nh_;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener tfListener_;

		ros::Publisher point_pub_;
		ros::Subscriber scan_sub_;
};

void Converter::initSetup() {
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &Converter::scanCallback, this);
	point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 100, false);
	tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void Converter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	sensor_msgs::PointCloud2 cloud;
	projector_.transformLaserScanToPointCloud("/laser", *scan, cloud, tfListener_);
	point_pub_.publish(cloud);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "converter");
	Converter Cv;
	Cv.initSetup();
	ros::spin();
	
	return 0;
}
