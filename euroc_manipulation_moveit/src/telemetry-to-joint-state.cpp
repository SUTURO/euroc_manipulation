//hyperspec
//gsll


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <euroc_c2_msgs/Telemetry.h>

using namespace std;

ros::Publisher joint_state_pub;

// 'axis_x', 'axis_y', 'lwr_joint_1', 'lwr_joint_2', 'lwr_joint_3', 'lwr_joint_4', 'lwr_joint_5', 'lwr_joint_6', 'lwr_joint_7', 'gripper', 'cam_pan', 'cam_tilt'
string map_joint(string joint)
{
    if (joint == "lwr_joint_1") return "joint1";
    if (joint == "lwr_joint_2") return "joint2";
    if (joint == "lwr_joint_3") return "joint3";
    if (joint == "lwr_joint_4") return "joint4";
    if (joint == "lwr_joint_5") return "joint5";
    if (joint == "lwr_joint_6") return "joint6";
    if (joint == "lwr_joint_7") return "joint7";
}

void callback(const euroc_c2_msgs::Telemetry::ConstPtr &telemetry)
{
    sensor_msgs::JointState joint_state;
    joint_state.header = telemetry->header;

    for (int i = 2; i < telemetry->joint_names.size() - 3; ++i)
    {
        joint_state.name.push_back(map_joint( telemetry->joint_names[i]));
        joint_state.position.push_back(telemetry->measured.position[i]);
        joint_state.velocity.push_back(telemetry->commanded.velocity[i]);
        joint_state.effort.push_back(telemetry->commanded.acceleration[i]);

    }
    joint_state.name.push_back("joint_before_finger2");
    joint_state.position.push_back(0);
    joint_state.velocity.push_back(0);
    joint_state.effort.push_back(0);

    joint_state.name.push_back("joint_before_finger1");
    joint_state.position.push_back(0);
    joint_state.velocity.push_back(0);
    joint_state.effort.push_back(0);

    joint_state_pub.publish(joint_state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "telemetry_to_joint_state");
    ros::NodeHandle n;

    joint_state_pub = n.advertise < sensor_msgs::JointState > ( "/joint_states", 10 );

    ros::Subscriber js_sub = n.subscribe("/euroc_interface_node/telemetry", 1000, callback);
    ROS_INFO_STREAM("hi");

    ros::spin();
    ros::waitForShutdown();
    return 0;
}












