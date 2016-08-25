// #include <iostream>
// #include <fstream>
// #include <chrono>
// #include <algorithm>
// #include <iterator>


// #include "pcl_ros/transforms.h"

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>



using namespace std;

ros::Publisher pcl_from_scan;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
laser_geometry::LaserProjection projector;

void hokuyo_callbacks(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud);

    cloud.header.frame_id = "/laser";
    cloud.header.stamp = scan_in->header.stamp;
    pcl_from_scan.publish(cloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserScan_to_pointcloud");
    ros::NodeHandle nh;

    ros::Subscriber hokuyo_sub;
    hokuyo_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_kinect", 1, hokuyo_callbacks);
    
    pcl_from_scan = nh.advertise<PointCloud>("hokuyo_points", 1);

    while (ros::ok())
    {
        ros::spin();
    }

    nh.shutdown();          
    return 0;
}
