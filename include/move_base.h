#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"


class MoveBase {
public:
    MoveBase(ros::NodeHandle _nh) : nh(_nh),
                                    pub(nh.advertise<geometry_msgs::Pose>("/mobile_base/commands/position", 10)) { }
    void move_to(float x, float y, float z, float yaw) {
        geometry_msgs::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();

        pub.publish(p);
    }

    void move_to(geometry_msgs::Pose &p) {
        pub.publish(p);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
};
