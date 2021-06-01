#include "ros/ros.h"
#include "iostream"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"

#include "vector"

#include "lidar_practice/polyfit.h"
#include "lidar_practice/polyfit.c"
using namespace std;

unsigned int ORDER = 3;
const double ACCEATABLE_ERROR = 0.01;

ros::Publisher pub, pub2;
float scan_data_[36] = {0};

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *cloud);
	cloud->header.frame_id = "laser";
	
	vector<pcl::PointXYZ> left_point, right_point;

	for (size_t i=0;i<cloud->points.size();i++) {
		if (cloud->points.at(i).x <= 6) {
			if (cloud->points.at(i).y < 0) {
				right_point.push_back(cloud->points.at(i));
			} else if (cloud->points.at(i).y > 0) {
				left_point.push_back(cloud->points.at(i));
			}		
		}
	}
	double left_coef[ORDER+1];
	double right_coef[ORDER+1];

	double left_xData[left_point.size()] = {0};
	double left_yData[left_point.size()] = {0};
	double right_xData[right_point.size()] = {0};
	double right_yData[right_point.size()] = {0};

	int left_result, right_result;

	for (int i=0;i<left_point.size();i++) {
		left_xData[i] = left_point.at(i).x;
		left_yData[i] = left_point.at(i).y;
	}
	
	for (int i=0;i<right_point.size();i++) {
		right_xData[i] = right_point.at(i).x;
		right_yData[i] = right_point.at(i).y;
	}

	left_result = polyfit(left_xData, left_yData, left_point.size(), ORDER, left_coef);
	right_result = polyfit(right_xData, right_yData, right_point.size(), ORDER, right_coef);

	cout << left_coef[3] << "x^3 + " << left_coef[2] << "x^2 + " << left_coef[1] << "x + " << left_coef[0] << endl << endl;
	cout << right_coef[3] << "x^3 + " << right_coef[2] << "x^2 + " << right_coef[1] << "x + " << right_coef[0] << endl << endl;

	left_point.clear();
	right_point.clear();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "lidar_node");	
	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 10);	
	pub2 = nh.advertise<std_msgs::Float32MultiArray>("graph", 10);	
	ros::Subscriber sub = nh.subscribe("cloud", 10, scanCallback);

	ros::spin();
	return 0;
}
