#ifndef JACKALEXPLORATION_EXPLORATION_H_
#define JACKALEXPLORATION_EXPLORATION_H_

// ROS related header files
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>

//C library headers:
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <unordered_set>


using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
const double PI = 3.1415926;
const double octo_reso = 0.2;


vector<pair<point3d, point3d>> candidates, visited_candidates;
unordered_set<int> visited;
vector<int> cidx;
long unsigned int num_of_candidates=1;


octomap::OcTree* cur_tree;
octomap_msgs::Octomap msg_octomap;
tf::TransformListener *tf_listener; 

point3d position, sensor_origin;

ofstream explo_log_file;
std::string octomap_name_2d, octomap_name_3d;
string map_frame="/map";

// Initialize parameters 
point3d Sensor_PrincipalAxis(1, 0, 0);
octomap::OcTree new_tree(octo_reso);


vector<int> sort_MIs(const vector<double> &v){
    vector<int> idx(v.size());
    iota(idx.begin(), idx.end(),0);

    sort(idx.begin(), idx.end(), 
        [&v](int i1, int i2) {return v[i1] > v[i2];});

    return idx;
}


struct sensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    string frame;
    // vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    sensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range, string _frame)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range), frame(_frame) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                InitialVector = point3d(1.0, 0.0, 0.0);
                InitialVector.rotate_IP(0.0, j * angle_inc_vel, i * angle_inc_hor);
                SensorRays.push_back(InitialVector);
        }
    }
}; 
// sensorModel mapping_sensor(360, 16, 2*PI, 0.5236, 100.0,"/velodyne");
sensorModel mapping_sensor(144, 144, PI, PI, 30.0,"/servo");


double countFreeVolume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}


octomap::Pointcloud castSensorRays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &sensor_orientation) {
    octomap::Pointcloud hits;

    octomap::Pointcloud RaysToCast;
    RaysToCast.push_back(mapping_sensor.SensorRays);
    RaysToCast.rotate(sensor_orientation.x(),sensor_orientation.y(),sensor_orientation.z());
    point3d end;
    // Cast Rays to 3d OctoTree and get hit points
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, mapping_sensor.max_range)) {
            hits.push_back(end);
        } else {
            end = RaysToCast.getPoint(i) * mapping_sensor.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}


double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    octree_copy->insertPointCloud(hits, sensor_orig, mapping_sensor.max_range, true, true);
    double after = countFreeVolume(octree_copy);
    delete octree_copy;
    return after - before;
}

void candidate_callbacks(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg) {

    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud);

    cidx.clear();
    candidates.clear();

    // for (int i = num_of_candidates; i< cloud->width; i++)
    for (int i = 1; i< cloud->width; i++)
    {
        for(int j=0;j<3;j++) {
            int idx = (i-1)*3+j;

            candidates.push_back(make_pair<point3d, point3d>
                (point3d(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z+0.3), 
                    point3d(0.0, 0.0, double(2*PI*j/3))));

            if(visited.find(idx) != visited.end()) continue;
            if(cloud->at(i).intensity == 0) continue;
            cidx.push_back(idx);
        }
        
    }

    num_of_candidates = cidx.size();

    //ROS_INFO("Got %ld candidate view points from PRM planner, idx at: %ld", candidates.size(), num_of_candidates);
    delete cloud;
}


#endif
