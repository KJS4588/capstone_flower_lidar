#include "ros/ros.h"
#include "iostream"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "std_msgs/Float32MultiArray.h"

#include "vector"
#include "cmath"

#include "lidar_practice/polyfit.h"
#include "lidar_practice/polyfit.c"
#define DIST   1
#define _USE_MATH_DEFINES

using namespace std;

const unsigned int ORDER = 2;
const double ACCEATABLE_ERROR = 0.01;

ros::Publisher pub, pub2, marker_pub, coef_pub;
pcl::PointXYZ ex_point;
float scan_data_[36] = {0};

double calcDist(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    double x = p1.x - p2.x;
    double y = p1.y - p2.y;
    
    return sqrt(x*x + y*y);
}

double calcplaneDist(double a, double b, pcl::PointXYZ tmp) {
    
    return abs(a*tmp.y - tmp.x + b)/sqrt(a*a + b*b);
}


void visualize(double coef[ORDER+1]) {
    visualization_msgs::Marker points;

	points.header.frame_id = "laser";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 1;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1; 
	points.scale.y = 0.1;
	points.color.a = 1.0;
	points.color.g = 1.0f;

	geometry_msgs::Point p;

	for (double i=1.5;i<5;i++) {
		p.y = coef[2]*i*i + coef[1]*i + coef[0];
        //coef[3]*i*i*i + coef[2]*i*i + coef[1]*i + coef[0];
		p.x = i;
		p.z = 0;
		points.points.push_back(p);
	}

	marker_pub.publish(points);

}

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    double ex_dist = 0.0;
    double dist1 = 0.0;
    double dist2 = 0.0;
    
    bool cnt = false;
    bool left_;
    std_msgs::Float32MultiArray way_coef;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *cloud);
	cloud->header.frame_id = "laser";
	
	vector<pcl::PointXYZ> left_point, right_point;
    pcl::PointXYZ tmp_p(0,0,0);
    int left_cnt = 0;
    int right_cnt = 0;
    for (size_t i=0;i<cloud->points.size();i++) {
        if (!cnt) {
            double degree = atan2(cloud->points.at(i).y, cloud->points.at(i).x) * 180 / M_PI;
            if (calcDist(cloud->points.at(i), tmp_p) < 6) {
                left_point.push_back(cloud->points.at(i));
                cnt = true;
            }
        } else {
            if (cloud->points.at(i).x <= 6) {
                dist1 = calcDist(cloud->points.at(i), left_point.at(left_point.size()-1));

                if (dist1 < DIST) {
                    left_point.push_back(cloud->points.at(i));
                } else if (dist1 > 2) {
                    double lx = left_point.at(0).x - left_point.at(left_point.size()-1).x;
                    double ly = left_point.at(0).y - left_point.at(left_point.size()-1).y;

                    double a = lx/ly;
                    double b = -a*left_point.at(0).y + left_point.at(0).x;

                    if (calcplaneDist(a, b, cloud->points.at(i)) > 1) right_point.push_back(cloud->points.at(i));
                }
            }
        }
    } 
    
    cout << right_point.size() << " " << left_point.size() << endl;
    int idx = right_point.size() > left_point.size() ? left_point.size() : right_point.size();
    pcl::PointCloud<pcl::PointXYZ> waypoint_;
    pcl::PointXYZ p;
    for (size_t i=0;i<idx;i++) {
        p.x = (left_point.at(i).x + right_point.at(i).x)/2;
        p.y = (left_point.at(i).y + right_point.at(i).y)/2;
        
        p.z = 0;        
        waypoint_.push_back(p);
    }  

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(waypoint_, cloud_p);
    
    sensor_msgs::PointCloud2 waypoint_pcl;
    pcl_conversions::fromPCL(cloud_p, waypoint_pcl);
    waypoint_pcl.header.frame_id = "laser";
    pub.publish(waypoint_pcl);

	double left_coef[ORDER+1];
	double right_coef[ORDER+1];
    double waypoint_coef[ORDER+1];

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
    
	//cout << left_coef[3] << "x^3 + " << left_coef[2] << "x^2 + " << left_coef[1] << "x + " << left_coef[0] << endl << endl;
	//cout << right_coef[3] << "x^3 + " << right_coef[2] << "x^2 + " << right_coef[1] << "x + " << right_coef[0] << endl << endl;
    
    for (int i=0;i<ORDER+1;i++) {
        waypoint_coef[i] = (left_coef[i] + right_coef[i])/2;       
        way_coef.data.push_back(waypoint_coef[i]);
    }

    visualize(waypoint_coef);
    
    coef_pub.publish(way_coef);

    left_point.clear();
	right_point.clear();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "lidar_node");	
	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::PointCloud2>("waypoint_mean", 10);	
	coef_pub = nh.advertise<std_msgs::Float32MultiArray>("coef", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/waypoint", 10);
	ros::Subscriber sub = nh.subscribe("cloud", 10, scanCallback);

	ros::spin();
	return 0;
}
