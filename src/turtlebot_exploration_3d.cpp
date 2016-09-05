#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>
#include <ctime>

#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include "navigation_utils.h"
#include <ros/callback_queue.h>


using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double octo_reso = 0.1;   // resolution of Octomap, unit: meters

octomap::OcTree* cur_tree;      // 3D Octomap for mapping
octomap::OcTree* cur_tree_2d;   // 2D Octomap for extracting frontiers and select candidates

tf::TransformListener *tf_listener; 
point3d laser_orig, velo_orig;  // sensor center

ofstream explo_log_file;        // log file for Map Entropy w.r.t. steps
std::string octomap_name_2d, octomap_name_3d;



struct SensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    SensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
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

// Establish sensor kinect
SensorModel Kinect_360(64, 48, 2*PI*57/360, 2*PI*43/360, 6);    // Construct sensor model : Kinect

//entropy Input: octree   Output:volume
double countFreeVolume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}

// Input: octree, position, direction. Output: hits
octomap::Pointcloud castSensorRays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    octomap::Pointcloud hits;

    octomap::Pointcloud SensorRays_copy;
    SensorRays_copy.push_back(Kinect_360.SensorRays);
    SensorRays_copy.rotate(0.0,0.0,direction.z());
    point3d end;
    // #pragma omp parallel for
    for(int i = 0; i < SensorRays_copy.size(); i++) {
        if(octree->castRay(position, SensorRays_copy.getPoint(i), end, true, Kinect_360.max_range)) {
            
        } else {
            end = SensorRays_copy.getPoint(i) * Kinect_360.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}

vector<vector<point3d>> extractFrontierPoints(const octomap::OcTree *octree) {

    vector<vector<point3d>> frontier_groups;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true; // whether or not a frontier point
    bool belong_old;//whether or not belong to old group
    double distance;
    double R1 = 0.4; //group length
    double x_cur, y_cur, z_cur;


    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

      if(!cur_tree_2d->isNodeOccupied(*n))
        {
         x_cur = n.getX();
         y_cur = n.getY();
         z_cur = n.getZ();
         //if there are unknown around the cube, the cube is frontier
         for (double x_cur_buf = x_cur - 0.1; x_cur_buf < x_cur + 0.15; x_cur_buf += octo_reso)
             for (double y_cur_buf = y_cur - 0.1; y_cur_buf < y_cur + 0.15; y_cur_buf += octo_reso)
            {
                n_cur_frontier = cur_tree_2d->search(point3d(x_cur_buf, y_cur_buf, z_cur));
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
vector<pair<point3d, point3d>> extractCandidateViewPoints(vector<vector<point3d>> frontier_groups, point3d sensor_orig ) {
    double R2 = 0.6;        // Robot step, in meters.
    double R3 = 0.25;       // to other frontiers
    double n = 6;
    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();
    double x, y;
    double yaw;
    double distance_can;

        for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++) {
            for(double yaw = 0; yaw < 2*PI; yaw += PI*2 / n){ 
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
                            for (double z_buf = sensor_orig.z()-0.2; z_buf <sensor_orig.z()+0.2; z_buf += octo_reso)
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




// Calculate Mutual Information. Input: octree, sensor_orig, hits, before
double calculateMutualInformation(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
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

    cur_tree->insertPointCloud(hits, velo_orig, Kinect_360.max_range);
    ROS_INFO("Entropy(3d map) : %f", countFreeVolume(cur_tree));

    cur_tree->write(octomap_name_3d);
    delete cloud;
    delete cloud_local;
}

void laserscanCallbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg )
{
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
    for (int j = 1; j< cloud->width; j++)
    {
        // if(isnan(cloud->at(j).x)) continue;
        hits.push_back(point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z));
    }
    cur_tree_2d->insertPointCloud(hits, laser_orig, 5.6);
    ROS_INFO("Entropy(2d map) : %f", countFreeVolume(cur_tree_2d));
    cur_tree_2d->write(octomap_name_2d);
    delete cloud;
    delete cloud_local;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "explo_sam_2d_turtlebot");
    ros::NodeHandle nh;

    // Initialize time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,80,"Trajectory_%R_%S_%m%d_DA.txt",timeinfo);
    std::string logfilename(buffer);
    std::cout << logfilename << endl;
    strftime(buffer,80,"octomap_2d_%R_%S_%m%d_DA.ot",timeinfo);
    octomap_name_2d = buffer;
    strftime(buffer,80,"octomap_3d_%R_%S_%m%d_DA.ot",timeinfo);
    octomap_name_3d = buffer;


    ros::Subscriber kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, kinectCallbacks);// need to change##########
    ros::Subscriber hokuyo_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hokuyo_points", 1, laserscanCallbacks);// need to change#############
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Goal_Marker", 1 );
    ros::Publisher JackalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Jackal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("Candidate_MIs", 1);
    ros::Publisher Octomap_marker_pub = nh.advertise<visualization_msgs::Marker>("Occupied_MarkerArray", 1);
    ros::Publisher Octomap_marker_3d_pub = nh.advertise<visualization_msgs::Marker>("Occupied_MarkerArray_3d", 1);
    ros::Publisher Frontier_points_pub = nh.advertise<visualization_msgs::Marker>("Frontier_points", 1);
    ros::Publisher Free_marker_pub = nh.advertise<visualization_msgs::Marker>("Free_MarkerArray", 1);
    ros::Publisher Free_marker_3d_pub = nh.advertise<visualization_msgs::Marker>("Free_MarkerArray_3d", 1);
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading; // robot's heading direction

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker OctomapOccupied_cubelist;
    visualization_msgs::Marker OctomapOccupied_cubelist_3d;
    visualization_msgs::Marker Frontier_points_cubelist;
    visualization_msgs::Marker Free_cubelist;
    visualization_msgs::Marker Free_cubelist_3d;
    geometry_msgs::Twist twist_cmd;



    ros::Time now_marker = ros::Time::now();
   
    double R_velo, P_velo, Y_velo;

    // Initialize parameters 
    int max_idx = 0;

    point3d Sensor_PrincipalAxis(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    octomap::OcTree new_tree_2d(octo_reso);
    cur_tree = &new_tree;
    cur_tree_2d = &new_tree_2d;
    point3d next_vp;

    bool got_tf = false;
    bool arrived;
    
    // Update the initial location of the robot
    for(int o =0; o < 6; o++){
        // Update the pose of the robot
        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
            velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: Kinect frame"); 
        } 
        ros::Duration(0.05).sleep();
        }

        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
            laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: LaserScan frame"); 
        } 
        ros::Duration(0.05).sleep();
        }

        // Take a Scan
        ros::spinOnce();

        // Rotate another 60 degrees
        twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
        ros::Time start_turn = ros::Time::now();

        ROS_WARN("Rotate 60 degrees");
        while (ros::Time::now() - start_turn < ros::Duration(2.6)){ // turning duration - second
        twist_cmd.angular.z = 0.6; // turning speed
        // turning angle = turning speed * turning duration / 3.14 * 180
        pub_twist.publish(twist_cmd);
        ros::Duration(0.05).sleep();
        }
        // stop
        twist_cmd.angular.z = 0;
        pub_twist.publish(twist_cmd);

    }

   

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: Kinect frame"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: LaserScan frame"); 
    } 
    ros::Duration(0.05).sleep();
    }
    // Take the last initial Scan
    ros::spinOnce();

    double train_time, test_time;

    // steps robot taken, counter
    int robot_step_counter = 0;

    while (ros::ok())
    {
            
            vector<vector<point3d>> frontier_groups=extractFrontierPoints( cur_tree_2d );
            
            //frontier_groups.clear();//in the next line
            unsigned long int o = 0;
            for(vector<vector<point3d>>::size_type e = 0; e < frontier_groups.size(); e++) {
                o = o+frontier_groups[e].size();
            }

            Frontier_points_cubelist.points.resize(o);
            ROS_INFO("frontier points %ld", o);
            now_marker = ros::Time::now();
            Frontier_points_cubelist.header.frame_id = "map";
            Frontier_points_cubelist.header.stamp = now_marker;
            Frontier_points_cubelist.ns = "frontier_points_array";
            Frontier_points_cubelist.id = 0;
            Frontier_points_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            Frontier_points_cubelist.action = visualization_msgs::Marker::ADD;
            Frontier_points_cubelist.scale.x = octo_reso;
            Frontier_points_cubelist.scale.y = octo_reso;
            Frontier_points_cubelist.scale.z = octo_reso;
            Frontier_points_cubelist.color.a = 1.0;
            Frontier_points_cubelist.color.r = (double)255/255;
            Frontier_points_cubelist.color.g = 0;
            Frontier_points_cubelist.color.b = (double)0/255;
            Frontier_points_cubelist.lifetime = ros::Duration();

            unsigned long int t = 0;
            int l = 0;
            geometry_msgs::Point q;
            for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++) { 
                for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
                   q.x = frontier_groups[n][m].x();
                   q.y = frontier_groups[n][m].y();
                   q.z = frontier_groups[n][m].z()+octo_reso;
                   Frontier_points_cubelist.points.push_back(q); 
                   
                }
                t++;
            }
            ROS_INFO("Publishing %ld frontier_groups", t);
            
            Frontier_points_pub.publish(Frontier_points_cubelist); //publish frontier_points
            Frontier_points_cubelist.points.clear();            

        // Generate Candidates
        vector<pair<point3d, point3d>> candidates = extractCandidateViewPoints(frontier_groups, laser_orig); 
        // Generate Testing poses
        ROS_INFO("%lu candidates generated.", candidates.size());
        frontier_groups.clear();

        while(candidates.size() < 1)
        {
            // Get the current heading
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: LaserScan frame");  
            } 
            ros::Duration(0.05).sleep();
            }
            // Rotate another 60 degrees
            twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
            ros::Time start_turn = ros::Time::now();

            ROS_WARN("Rotate 60 degrees");
            while (ros::Time::now() - start_turn < ros::Duration(2.6)){ // turning duration - second
            twist_cmd.angular.z = 0.4; // turning speed
            pub_twist.publish(twist_cmd);
            ros::Duration(0.05).sleep();
            }
            // stop
            twist_cmd.angular.z = 0;
            pub_twist.publish(twist_cmd);
            vector<pair<point3d, point3d>> candidates = extractCandidateViewPoints(frontier_groups, laser_orig);
        }
        
        vector<double> MIs(candidates.size());
        double before = countFreeVolume(cur_tree);
        max_idx = 0;

        unsigned int p = 0;

        // for every candidate...
        double Secs_CastRay, Secs_InsertRay, Secs_tmp;  
        Secs_InsertRay = 0;
        Secs_CastRay = 0;

        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {   
            //max_order[i] = i;
            auto c = candidates[i];
            // Evaluate Mutual Information
            Secs_tmp = ros::Time::now().toSec();
            Sensor_PrincipalAxis.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            octomap::Pointcloud hits = castSensorRays(cur_tree, c.first, Sensor_PrincipalAxis);  
            Secs_CastRay += ros::Time::now().toSec() - Secs_tmp;
            Secs_tmp = ros::Time::now().toSec();
            MIs[i] = calculateMutualInformation(cur_tree, c.first, hits, before)/pow(  pow(c.first.x()-laser_orig.x(), 2) + pow(c.first.y() - laser_orig.y(), 2)  ,1.5);
            Secs_InsertRay += ros::Time::now().toSec() - Secs_tmp;
        }
      
        // ###########################
        long int max_order[candidates.size()];

        for(int j=0; j<candidates.size(); j++)
           {
            p=0;
           for(int m=0; m<candidates.size(); m++)
              {
            
              if (MIs[j] > MIs[m])
                 {
                 p++;
                 }
              }
           max_order[p] = j;
           }
        
        p = candidates.size()-1;
        max_idx = max_order[p];
        loop:
        //max_idx = max_order[p];
 
        next_vp = point3d(candidates[max_order[p]].first.x(),candidates[max_order[p]].first.y(),candidates[max_order[p]].first.z());
        Goal_heading.setRPY(0.0, 0.0, candidates[max_order[p]].second.yaw());
        Goal_heading.normalize();
        ROS_INFO("Estimated Max MI : %f , @ %3.2f,  %3.2f,  %3.2f", MIs[max_order[p]], next_vp.x(), next_vp.y(), next_vp.z() );
        ROS_INFO("CastRay Time: %2.3f Secs. InsertRay Time: %2.3f Secs.", Secs_CastRay, Secs_InsertRay);

        // Publish the candidates as marker array in rviz
        tf::Quaternion MI_heading;
        MI_heading.setRPY(0.0, -PI/2, 0.0);
        MI_heading.normalize();
        
        CandidatesMarker_array.markers.resize(candidates.size());
        for (int i = 0; i < candidates.size(); i++)
        {
            CandidatesMarker_array.markers[i].header.frame_id = "map";
            CandidatesMarker_array.markers[i].header.stamp = ros::Time::now();
            CandidatesMarker_array.markers[i].ns = "candidates";
            CandidatesMarker_array.markers[i].id = i;
            CandidatesMarker_array.markers[i].type = visualization_msgs::Marker::ARROW;
            CandidatesMarker_array.markers[i].action = visualization_msgs::Marker::ADD;
            CandidatesMarker_array.markers[i].pose.position.x = candidates[i].first.x();
            CandidatesMarker_array.markers[i].pose.position.y = candidates[i].first.y();
            CandidatesMarker_array.markers[i].pose.position.z = candidates[i].first.z();
            CandidatesMarker_array.markers[i].pose.orientation.x = MI_heading.x();
            CandidatesMarker_array.markers[i].pose.orientation.y = MI_heading.y();
            CandidatesMarker_array.markers[i].pose.orientation.z = MI_heading.z();
            CandidatesMarker_array.markers[i].pose.orientation.w = MI_heading.w();
            CandidatesMarker_array.markers[i].scale.x = (double)MIs[i]/MIs[max_idx];
            CandidatesMarker_array.markers[i].scale.y = 0.05;
            CandidatesMarker_array.markers[i].scale.z = 0.05;
            CandidatesMarker_array.markers[i].color.a = (double)MIs[i]/MIs[max_idx];
            CandidatesMarker_array.markers[i].color.r = 0.0;
            CandidatesMarker_array.markers[i].color.g = 1.0;
            CandidatesMarker_array.markers[i].color.b = 0.0;
        }
        Candidates_pub.publish(CandidatesMarker_array); //publish candidates##########
        CandidatesMarker_array.markers.clear();
        //candidates.clear();

        // Publish the goal as a Marker in rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = next_vp.x();
        marker.pose.position.y = next_vp.y();
        marker.pose.position.z = 1.0;
        marker.pose.orientation.x = Goal_heading.x();
        marker.pose.orientation.y = Goal_heading.y();
        marker.pose.orientation.z = Goal_heading.z();
        marker.pose.orientation.w = Goal_heading.w();
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        GoalMarker_pub.publish( marker ); //publish goal##########

        // Send the Robot 
        Goal_heading.setRPY(0.0, 0.0, candidates[max_order[p]].second.yaw());
        arrived = goToDest(next_vp, Goal_heading);

        if(arrived)
        {
            // Update the initial location of the robot
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
                velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: Kinect frame"); 
            } 
            ros::Duration(0.05).sleep();
            }

            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: LaserScan frame"); 
            } 
            ros::Duration(0.05).sleep();
            }

            // Update Octomap
            ros::spinOnce();
            ROS_INFO("Updating to Octomap, map volume(Free): %f", countFreeVolume(cur_tree));
            robot_step_counter++;

            // Prepare the header for occupied array
            now_marker = ros::Time::now();
            OctomapOccupied_cubelist.header.frame_id = "map";
            OctomapOccupied_cubelist.header.stamp = now_marker;
            OctomapOccupied_cubelist.ns = "octomap_occupied_array";
            OctomapOccupied_cubelist.id = 0;
            OctomapOccupied_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            OctomapOccupied_cubelist.action = visualization_msgs::Marker::ADD;
            OctomapOccupied_cubelist.scale.x = octo_reso;
            OctomapOccupied_cubelist.scale.y = octo_reso;
            OctomapOccupied_cubelist.scale.z = octo_reso;
            OctomapOccupied_cubelist.color.a = 0.5;
            OctomapOccupied_cubelist.color.r = (double)19/255;
            OctomapOccupied_cubelist.color.g = (double)121/255;
            OctomapOccupied_cubelist.color.b = (double)156/255;

            unsigned long int j = 0;
            geometry_msgs::Point p;
            for(octomap::OcTree::leaf_iterator n = cur_tree_2d->begin_leafs(cur_tree_2d->getTreeDepth()); n != cur_tree_2d->end_leafs(); ++n) { // changed there#######
                if(!cur_tree_2d->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                OctomapOccupied_cubelist.points.push_back(p); 
                j++;
            }
            ROS_INFO("Updating %ld 2D occupied cells in RVIZ", j);
            Octomap_marker_pub.publish(OctomapOccupied_cubelist); //publish octomap############

              // Prepare the header for 2d occupied array
              now_marker = ros::Time::now();
              OctomapOccupied_cubelist_3d.header.frame_id = "map";
              OctomapOccupied_cubelist_3d.header.stamp = now_marker;
              OctomapOccupied_cubelist_3d.ns = "octomap_occupied_array";
              OctomapOccupied_cubelist_3d.id = 0;
              OctomapOccupied_cubelist_3d.type = visualization_msgs::Marker::CUBE_LIST;
              OctomapOccupied_cubelist_3d.action = visualization_msgs::Marker::ADD;
              OctomapOccupied_cubelist_3d.scale.x = octo_reso;
              OctomapOccupied_cubelist_3d.scale.y = octo_reso;
              OctomapOccupied_cubelist_3d.scale.z = octo_reso;
              OctomapOccupied_cubelist_3d.color.a = 0.5;
              OctomapOccupied_cubelist_3d.color.r = (double)19/255;
              OctomapOccupied_cubelist_3d.color.g = (double)121/255;
              OctomapOccupied_cubelist_3d.color.b = (double)156/255;
  
              j = 0;
              for(octomap::OcTree::leaf_iterator n = cur_tree->begin_leafs(cur_tree->getTreeDepth()); n != cur_tree->end_leafs(); ++n) {
                  if(!cur_tree->isNodeOccupied(*n)) continue;
                  p.x = n.getX();
                  p.y = n.getY();
                  p.z = n.getZ();
                  OctomapOccupied_cubelist_3d.points.push_back(p); 
                  j++;
              }
              ROS_INFO("Updating %ld 3D occupied cells in RVIZ", j);
              Octomap_marker_3d_pub.publish(OctomapOccupied_cubelist_3d); //publish octomap############

            // Prepare the header for free array
            now_marker = ros::Time::now();
            Free_cubelist.header.frame_id = "map";
            Free_cubelist.header.stamp = now_marker;
            Free_cubelist.ns = "octomap_free_array";
            Free_cubelist.id = 0;
            Free_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            Free_cubelist.action = visualization_msgs::Marker::ADD;
            Free_cubelist.scale.x = octo_reso;
            Free_cubelist.scale.y = octo_reso;
            Free_cubelist.scale.z = octo_reso;
            Free_cubelist.color.a = 0.3;
            Free_cubelist.color.r = (double)102/255;
            Free_cubelist.color.g = (double)255/255;
            Free_cubelist.color.b = (double)102/255;

            j = 0;
            
            for(octomap::OcTree::leaf_iterator n = cur_tree_2d->begin_leafs(cur_tree_2d->getTreeDepth()); n != cur_tree_2d->end_leafs(); ++n) { // changed there#######
                if(cur_tree_2d->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                Free_cubelist.points.push_back(p); 
                j++;
            }
            ROS_INFO("Updating %ld free cells in RVIZ", j);
            Free_marker_pub.publish(Free_cubelist); //publish octomap############
            //unsigned long int g;

           // Prepare the header for 3D free array
            now_marker = ros::Time::now();
            Free_cubelist_3d.header.frame_id = "map";
            Free_cubelist_3d.header.stamp = now_marker;
            Free_cubelist_3d.ns = "octomap_free_array";
            Free_cubelist_3d.id = 0;
            Free_cubelist_3d.type = visualization_msgs::Marker::CUBE_LIST;
            Free_cubelist_3d.action = visualization_msgs::Marker::ADD;
            Free_cubelist_3d.scale.x = octo_reso;
            Free_cubelist_3d.scale.y = octo_reso;
            Free_cubelist_3d.scale.z = octo_reso;
            Free_cubelist_3d.color.a = 0.3;
            Free_cubelist_3d.color.r = (double)102/255;
            Free_cubelist_3d.color.g = (double)255/255;
            Free_cubelist_3d.color.b = (double)102/255;

            j = 0;
            //geometry_msgs::Point p;
            for(octomap::OcTree::leaf_iterator n = cur_tree->begin_leafs(cur_tree_2d->getTreeDepth()); n != cur_tree->end_leafs(); ++n) { // changed there#######
                if(cur_tree->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                Free_cubelist_3d.points.push_back(p); 
                j++;
            }
            ROS_INFO("Updating %ld free cells in RVIZ", j);
            Free_marker_3d_pub.publish(Free_cubelist_3d); //publish octomap############

            // Send out results to file.
            explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
            explo_log_file << "Robot Step(DA): " << robot_step_counter << "  - Current Entropy: " << countFreeVolume(cur_tree) << endl;
            explo_log_file.close();

        }
        else
        {
            ROS_ERROR("Cannot navigate to the best view point, switch to a second best...");
            p--;
            if(p < 0){
               continue;
            }

            goto loop;
        }
    }
    nh.shutdown();          
    return 0;
}
