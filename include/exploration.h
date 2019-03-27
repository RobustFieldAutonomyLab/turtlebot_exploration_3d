#ifndef JACKALEXPLORATION_EXPLORATION_H_
#define JACKALEXPLORATION_EXPLORATION_H_

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

#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <numeric>

using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double octo_reso = 0.05;
const int num_of_samples_eva = 15;
const int num_of_bay = 3;


octomap::OcTree* cur_tree;
octomap::OcTree* cur_tree_2d;
octomap_msgs::Octomap msg_octomap;

tf::TransformListener *tf_listener; 

point3d kinect_orig;

ofstream explo_log_file;
std::string octomap_name_3d;

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
    // vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    sensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
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
sensorModel Kinect_360(128, 96, 2*PI*57/360, 2*PI*43/360, 6);    // Construct sensor model : Kinect


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
    RaysToCast.push_back(Kinect_360.SensorRays);
    RaysToCast.rotate(sensor_orientation.x(),sensor_orientation.y(),sensor_orientation.z());
    point3d end;
    // Cast Rays to 3d OctoTree and get hit points
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, Kinect_360.max_range)) {
            hits.push_back(end);
        } else {
            end = RaysToCast.getPoint(i) * Kinect_360.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}

// extract 2d frontier points
vector<vector<point3d>> extractFrontierPoints(const octomap::OcTree *octree) {

    vector<vector<point3d>> frontier_groups;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true;         // whether or not a frontier point
    bool belong_old;            //whether or not belong to old group
    double distance;
    double R1 = 0.4;            //group size
    double x_cur, y_cur, z_cur;


    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

      if(!octree->isNodeOccupied(*n))
        {
         x_cur = n.getX();
         y_cur = n.getY();
         z_cur = n.getZ();

         if(z_cur < 0.4)    continue;
         if(z_cur > 0.4 + octo_reso)    continue;
         //if there are unknown around the cube, the cube is frontier
         for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
             for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
            {
                n_cur_frontier = octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                if(!n_cur_frontier)
                {
                    frontier_true = true;
                    continue;            
                }

            }
            if(frontier_true)// && num_free >5 )
            {
                // divede frontier points into groups
                if(frontier_groups.size() < 1)
                {
                    frontier_points.resize(1);
                    frontier_points[0] = point3d(x_cur,y_cur,z_cur);
                    frontier_groups.push_back(frontier_points);
                    frontier_points.clear();
                }
                else
                {
                    bool belong_old = false;            

                    for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                            distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)) ;
                            if(distance < R1){
                               frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
                               belong_old = true;
                               break;
                            }
                    }
                    if(!belong_old){
                               frontier_points.resize(1);
                               frontier_points[0] = point3d(x_cur, y_cur, z_cur);
                               frontier_groups.push_back(frontier_points);
                               frontier_points.clear();
                    }                              
                }

            } 
        }
        
    }
    return frontier_groups;
}

//generate candidates for moving. Input sensor_orig and initial_yaw, Output candidates
//senor_orig: locationg of sensor.   initial_yaw: yaw direction of sensor
vector<pair<point3d, point3d>> extractCandidateViewPoints(vector<vector<point3d>> frontier_groups, point3d sensor_orig, int n ) {
    double R2_min = 1.0;        // distance from candidate view point to frontier centers, in meters.
    double R2_max = 5.0;
    double R3 = 0.3;        // to other frontiers

    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();
    double x, y, yaw, distance_can;

        for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++) {
            for(double yaw = 0; yaw < 2*PI; yaw += PI*2/n )
                for(double R2 = R2_min; R2<=R2_max; R2+=0.5) { 
                x = frontier_groups[u][0].x() - R2 * cos(yaw);
                y = frontier_groups[u][0].y() - R2 * sin(yaw);

                bool candidate_valid = true;
                n_cur_3d = cur_tree->search(point3d(x, y, z));


                if (!n_cur_3d) {
                    candidate_valid = false;
                    continue;
                }

                if(sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2)) < 0.25){
                  candidate_valid = false;// delete candidates close to sensor_orig
                  continue;
                }

                else{

                    // check candidate to other frontiers;
                    for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++)
                        for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
                            distance_can = sqrt(pow(x - frontier_groups[n][m].x(),2) + pow(y - frontier_groups[n][m].y(),2));
                            if(distance_can < R3){
                                candidate_valid = false;        //delete candidates close to frontier
                                continue;
                            }
                    }
                
                    // volumn check
                    for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                        for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                            for (double z_buf = sensor_orig.z()-0.1; z_buf <sensor_orig.z()+0.3; z_buf += octo_reso)
                            {
                                n_cur_3d = cur_tree->search(point3d(x_buf, y_buf, z_buf));
                                if(!n_cur_3d)       continue;
                                else if (cur_tree->isNodeOccupied(n_cur_3d)){
                                candidate_valid = false;//delete candidates which have ccupied cubes around in 3D area
                                continue;
                                }  
                            }

                }

                if (candidate_valid)
                {
                    candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                }
            }
        }
    return candidates;
}


double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    octree_copy->insertPointCloud(hits, sensor_orig, Kinect_360.max_range, true, true);
    double after = countFreeVolume(octree_copy);
    delete octree_copy;
    return after - before;
}


void kinectCallbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);
    octomap::Pointcloud hits;

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
    }
    // Insert points into octomap one by one...
    for (int i = 1; i< cloud->width; i++)
    {
        for (int j = 1; j< cloud->height; j++)
        {
            if(isnan(cloud->at(i,j).x))     continue;
            if(cloud->at(i,j).z < -1.0)     continue;  
            hits.push_back(point3d(cloud->at(i,j).x, cloud->at(i,j).y, cloud->at(i,j).z));
        }
    }

    cur_tree->insertPointCloud(hits, kinect_orig, Kinect_360.max_range);
    
    cur_tree->write(octomap_name_3d);
    ROS_INFO("Entropy(3d map) : %f", countFreeVolume(cur_tree));

    delete cloud;
    delete cloud_local;
}


#endif
