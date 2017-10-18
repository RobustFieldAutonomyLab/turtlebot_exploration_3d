// Related headers:
#include "exploration.h"


//C++ library headers:  NONE


//other library headers:  NONE


using namespace std;

int robot_step_counter = 0;
// char buffer[80];
// std::string logfilename(buffer);

void viewpoints(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg) {

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
    ROS_INFO("got bona_pointcloud with %ld view points", num_of_candidates);
    delete cloud;
}


void octomaps(const octomap_msgs::OctomapConstPtr& map_msg) {
    // octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    // ref: http://www.cnblogs.com/shhu1993/p/7062099.html
    cur_tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*map_msg));
	// auto octree_copy = new octomap::OcTree(&octomap::binaryMsgToMap(map_msg));
	ROS_INFO("got octomap message");

    // Evaluate MI for every candidate view points
    int num_evaluations = candidates.size();
    vector<double>  MIs(num_evaluations, 0.0);
    double before = countFreeVolume(cur_tree);

    // open the file for training samples
    string logfilename = "Offline_step_" + to_string(robot_step_counter++);
    explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
    
    ROS_INFO("evaluating Mutual Information for %d candidate view points", num_evaluations);

    #pragma omp parallel for
    for(int i = 0; i < num_evaluations; i++) {
        auto c = candidates[i];
        // Evaluate Mutual Information
        Sensor_PrincipalAxis = point3d(1.0, 0.0, 0.0);
        Sensor_PrincipalAxis.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
        octomap::Pointcloud hits = castSensorRays(cur_tree, c.first, Sensor_PrincipalAxis);
        
        // Considering pure MI for decision making
        MIs[i] = calc_MI(cur_tree, c.first, hits, before);
        
        // write training sample and information gain to file
        explo_log_file << c.first.x() << ", " << c.first.y() << ", " << c.first.z() << ", "
                           << c.second.x() << ", " << c.second.y() << ", " << c.second.z() << ", "
                           << MIs[i] << endl;
    }

    explo_log_file.close();

    cout << logfilename << " saved." << endl;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "tuning_hyper_parameters");
    ros::NodeHandle nh;

    ros::Subscriber candidate_sub = nh.subscribe<sensor_msgs::PointCloud2>("/bona_pointcloud", 1, viewpoints);
    
    ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_3d", 1, octomaps);


    while(ros::ok()) {
    	ros::spinOnce();
    	ros::Duration(0.5).sleep();
    }

}




































