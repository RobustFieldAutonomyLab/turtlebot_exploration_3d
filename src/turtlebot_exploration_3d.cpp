// Related headers:
#include "exploration.h"
#include "navigation_utils.h"
#include "gpregressor.h"
#include "covMaterniso3.h"

//C library headers:
#include <iostream>
#include <fstream>
// #include <chrono>
// #include <iterator>
// #include <ctime>

//C++ library headers:  NONE
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//other library headers:  NONE


using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot_exploration_3d");
    ros::NodeHandle nh;

    // Initialize time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);


    strftime(buffer,80,"Octomap3D_%m%d_%R.ot",timeinfo);
    octomap_name_3d = buffer;


    ros::Subscriber kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, kinectCallbacks);// need to change##########
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "/Goal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("/Candidate_MIs", 1);
    ros::Publisher Frontier_points_pub = nh.advertise<visualization_msgs::Marker>("/Frontier_points", 1);
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    ros::Publisher Octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_3d",1);


    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading; // robot's heading direction

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker Frontier_points_cubelist;
    geometry_msgs::Twist twist_cmd;

    ros::Time now_marker = ros::Time::now();
   
    // Initialize parameters 
    int max_idx = 0;

    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    cur_tree = &new_tree;
    point3d next_vp;

    bool got_tf = false;
    bool arrived;
    
    // Update the initial location of the robot
    for(int i =0; i < 6; i++){
        // Update the pose of the robot
        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
            kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: Kinect frame"); 
        } 
        ros::Duration(0.05).sleep();
        }

        // Take a Scan
        ros::spinOnce();

        // prepare octomap msg
        octomap_msgs::binaryMapToMsg(*cur_tree, msg_octomap);
        msg_octomap.binary = 1;
        msg_octomap.id = 1;
        msg_octomap.resolution = octo_reso;
        msg_octomap.header.frame_id = "/map";
        msg_octomap.header.stamp = ros::Time::now();
        Octomap_pub.publish(msg_octomap);

        // Rotate another 60 degrees
        twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
        ros::Time start_turn = ros::Time::now();

        ROS_WARN("Rotate...");
        while (ros::Time::now() - start_turn < ros::Duration(3.0)){ // turning duration - second
        twist_cmd.angular.z = 0.6; // turning speed
        // turning angle = turning speed * turning duration / 3.14 * 180
        pub_twist.publish(twist_cmd);
        ros::Duration(0.05).sleep();
        }
        // stop
        twist_cmd.angular.z = 0;
        pub_twist.publish(twist_cmd);

    }

    // steps robot taken, counter
    int robot_step_counter = 0;

    while (ros::ok())
    {
        vector<vector<point3d>> frontier_groups=extractFrontierPoints(cur_tree);
        
        //frontier_groups.clear();//in the next line
        unsigned long int o = 0;
        for(vector<vector<point3d>>::size_type e = 0; e < frontier_groups.size(); e++) {
            o = o+frontier_groups[e].size();
        }

        Frontier_points_cubelist.points.resize(o);
        Frontier_points_cubelist.header.frame_id = "map";
        Frontier_points_cubelist.header.stamp = ros::Time::now();
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
        
        Frontier_points_pub.publish(Frontier_points_cubelist); //publish frontier_points
        Frontier_points_cubelist.points.clear();           

        // Generate Candidates
        vector<pair<point3d, point3d>> candidates = extractCandidateViewPoints(frontier_groups, kinect_orig, 30); 
        std::random_shuffle(candidates.begin(),candidates.end()); // shuffle to select a subset
        vector<pair<point3d, point3d>> gp_test_poses = candidates;
        ROS_INFO("Candidate View Points: %lu Genereated, %d evaluating...", candidates.size(), num_of_samples_eva);
        int temp_size = candidates.size()-3;
        if (temp_size < 1) {
            ROS_ERROR("Very few candidates generated, finishing with exploration...");
            nh.shutdown();
            return 0;
        }

        // Generate Testing poses
        candidates.resize(min(num_of_samples_eva,temp_size));
        frontier_groups.clear();

        // Evaluate MI for every candidate view points
        vector<double>  MIs(candidates.size());
        double before = countFreeVolume(cur_tree);
        // int max_idx = 0;
        double begin_mi_eva_secs, end_mi_eva_secs;
        begin_mi_eva_secs = ros::Time::now().toSec();

        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            // Evaluate Mutual Information
            octomap::Pointcloud hits = castSensorRays(cur_tree, c.first, c.second);
            
            // Considering pure MI for decision making
            // MIs[i] = calc_MI(cur_tree, c.first, hits, before);
            
            // Normalize the MI with distance
            MIs[i] = calc_MI(cur_tree, c.first, hits, before) / 
                sqrt(pow(c.first.x()-kinect_orig.x(),2) + pow(c.first.y()-kinect_orig.y(),2));

            // Pick the Candidate view point with max MI
            // if (MIs[i] > MIs[max_idx])
            // {
            //     max_idx = i;
            // }
        }


        // Bayesian Optimization for actively selecting candidate
        double train_time, test_time;
        GPRegressor g(100, 3, 0.01);
        for (int bay_itr = 0; bay_itr < num_of_bay; bay_itr++) {
            //Initialize gp regression
            
            MatrixXf gp_train_x(candidates.size(), 3), gp_train_label(candidates.size(), 1), gp_test_x(gp_test_poses.size(), 3);

            for (int i=0; i< candidates.size(); i++){
                gp_train_x(i,0) = candidates[i].first.x();
                gp_train_x(i,1) = candidates[i].first.y();
                gp_train_x(i,2) = candidates[i].second.z();
                gp_train_label(i) = MIs[i];
            }

            for (int i=0; i< gp_test_poses.size(); i++){
                gp_test_x(i,0) = gp_test_poses[i].first.x();
                gp_test_x(i,1) = gp_test_poses[i].first.y();
                gp_test_x(i,2) = gp_test_poses[i].second.z();
            }

            // Perform GP regression
            MatrixXf gp_mean_MI, gp_var_MI;
            train_time = ros::Time::now().toSec();
            g.train(gp_train_x, gp_train_label);
            train_time = ros::Time::now().toSec() - train_time;

            test_time = ros::Time::now().toSec();
            g.test(gp_test_x, gp_mean_MI, gp_var_MI);
            test_time = ros::Time::now().toSec() - test_time;

            // Get Acquisition function
            double beta = 2.4;
            vector<double>  bay_acq_fun(gp_test_poses.size());
            for (int i = 0; i < gp_test_poses.size(); i++) {
                bay_acq_fun[i] = gp_mean_MI(i) + beta*gp_var_MI(i);
            }
            vector<int> idx_acq = sort_MIs(bay_acq_fun);

            // evaluate MI, add to the candidate
            auto c = gp_test_poses[idx_acq[0]];
            octomap::Pointcloud hits = castSensorRays(cur_tree, c.first, c.second);
            candidates.push_back(c);
            MIs.push_back(calc_MI(cur_tree, c.first, hits, before));
            gp_test_poses.erase(gp_test_poses.begin()+idx_acq[0]);
        }
        
        end_mi_eva_secs = ros::Time::now().toSec();
        ROS_INFO("Mutual Infomation Eva took:  %3.3f Secs.", end_mi_eva_secs - begin_mi_eva_secs);

        // Normalize the MI with distance
        // for(int i = 0; i < candidates.size(); i++) {
        //     auto c = candidates[i];
        //     MIs[i] = MIs[i] / 
        //         sqrt(pow(c.first.x()-kinect_orig.x(),2) + pow(c.first.y()-kinect_orig.y(),2));
        // }

        // sort vector MIs, with idx_MI, descending
        vector<int> idx_MI = sort_MIs(MIs);

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
            CandidatesMarker_array.markers[i].scale.x = (double)2.0*MIs[i]/MIs[idx_MI[0]];
            CandidatesMarker_array.markers[i].scale.y = 0.2;
            CandidatesMarker_array.markers[i].scale.z = 0.2;
            CandidatesMarker_array.markers[i].color.a = (double)MIs[i]/MIs[idx_MI[0]];
            CandidatesMarker_array.markers[i].color.r = 1.0;
            CandidatesMarker_array.markers[i].color.g = 0.55;
            CandidatesMarker_array.markers[i].color.b = 0.22;
        }
        Candidates_pub.publish(CandidatesMarker_array);
        CandidatesMarker_array.markers.clear();
        candidates.clear();

        // loop in the idx_MI, if the candidate with max MI cannot be achieved, 
        // switch to a sub-optimal MI.
        arrived = false;
        int idx_ptr = 0;

        while (!arrived) {
            // Setup the Goal
            next_vp = point3d(candidates[idx_MI[idx_ptr]].first.x(),candidates[idx_MI[idx_ptr]].first.y(),candidates[idx_MI[idx_ptr]].first.z());
            Goal_heading.setRPY(0.0, 0.0, candidates[idx_MI[idx_ptr]].second.yaw());
            Goal_heading.normalize();
            ROS_INFO("Max MI : %f , @ location: %3.2f  %3.2f  %3.2f", MIs[idx_MI[idx_ptr]], next_vp.x(), next_vp.y(), next_vp.z() );
            
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
            marker.scale.x = 1.0;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            GoalMarker_pub.publish( marker );

            // Send the Robot 
            arrived = goToDest(next_vp, Goal_heading);

            if(arrived)
            {
                // Update the initial location of the robot
                got_tf = false;
                while(!got_tf){
                try{
                    tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
                    kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                    got_tf = true;
                }
                catch (tf::TransformException ex) {
                    ROS_WARN("Wait for tf: Kinect frame"); 
                } 
                ros::Duration(0.05).sleep();
                }
                // Update Octomap
                ros::spinOnce();
                ROS_INFO("Succeed, new Map Free Volume: %f", countFreeVolume(cur_tree));
                robot_step_counter++;

                // prepare octomap msg
                octomap_msgs::binaryMapToMsg(*cur_tree, msg_octomap);
                msg_octomap.binary = 1;
                msg_octomap.id = 1;
                msg_octomap.resolution = octo_reso;
                msg_octomap.header.frame_id = "/map";
                msg_octomap.header.stamp = ros::Time::now();
                Octomap_pub.publish(msg_octomap);

            }
            else
            {
                ROS_WARN("Failed to drive to the %d th goal, switch to the sub-optimal..", idx_ptr);
                idx_ptr++;
                if(idx_ptr > MIs.size()) {
                    ROS_ERROR("None of the goal is valid for path planning, shuting down the node");
                    nh.shutdown();
                }
            }

        }

        
        // r.sleep();
    }
    nh.shutdown();          
    return 0;
}
