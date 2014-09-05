#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <euroc_c2_msgs/Telemetry.h>
#include <tf/transform_broadcaster.h>


#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

using namespace std;

ros::Publisher joint_state_pub;

tf::TransformBroadcaster *br;
tf::TransformBroadcaster *br2;

// 'axis_x', 'axis_y', 'lwr_joint_1', 'lwr_joint_2', 'lwr_joint_3', 'lwr_joint_4', 'lwr_joint_5', 'lwr_joint_6', 'lwr_joint_7', 'gripper', 'cam_pan', 'cam_tilt'
// string map_joint(string joint)
//

void publishTfFrame(std::string frame_id, geometry_msgs::PoseStamped pose, tf::TransformBroadcaster br)
{

    ROS_DEBUG_STREAM("Publish TF frame " << frame_id);
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.pose.position.x,    pose.pose.position.y, pose.pose.position.z) );

    transform.setRotation( tf::Quaternion(pose.pose.orientation.x,
                                          pose.pose.orientation.y,
                                          pose.pose.orientation.z,
                                          pose.pose.orientation.w) );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), pose.header.frame_id, frame_id));
}

void publish_cam_frames()
{
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "/pt";
    ps.pose.position.x = 0.2;
    ps.pose.position.y = 0.02;
    ps.pose.orientation.w = 1;
    // ps.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(-M_PI_2,  0.0,  -M_PI_2);
    publishTfFrame("/srgb", ps, *br);

    ps.header.frame_id = "/srgb";
    ps.pose.position.x = 0;//0.04;
    ps.pose.position.y = -0.04;
    // ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    publishTfFrame("/sdepth", ps, *br);
}

void callback(const euroc_c2_msgs::Telemetry::ConstPtr &telemetry)
{
    sensor_msgs::JointState joint_state;
    joint_state.header = telemetry->header;

    for (int i = 2; i < telemetry->joint_names.size() - 3; ++i)
    {
        joint_state.name.push_back(telemetry->joint_names[i]);
        joint_state.position.push_back(telemetry->measured.position[i]);
        joint_state.velocity.push_back(telemetry->commanded.velocity[i]);
        joint_state.effort.push_back(telemetry->commanded.acceleration[i]);

    }
    joint_state.name.push_back("joint_before_finger2");
    joint_state.position.push_back(telemetry->measured.position[9] / 2);
    joint_state.velocity.push_back(telemetry->commanded.velocity[9]);
    joint_state.effort.push_back(telemetry->commanded.acceleration[9]);

    joint_state.name.push_back("joint_before_finger1");
    joint_state.position.push_back(-telemetry->measured.position[9] / 2);
    joint_state.velocity.push_back(telemetry->commanded.velocity[9]);
    joint_state.effort.push_back(telemetry->commanded.acceleration[9]);

    joint_state.name.push_back("base_axis_x");
    joint_state.position.push_back(telemetry->measured.position[3]);
    joint_state.velocity.push_back(telemetry->commanded.velocity[9]);
    joint_state.effort.push_back(telemetry->commanded.acceleration[9]);

    joint_state.name.push_back("base_axis_y");
    joint_state.position.push_back(telemetry->measured.position[3]);
    joint_state.velocity.push_back(telemetry->commanded.velocity[2]);
    joint_state.effort.push_back(telemetry->commanded.acceleration[2]);

    joint_state_pub.publish(joint_state);

    geometry_msgs::PoseStamped cam_pose;
    cam_pose.header.frame_id = "/pt_base";
    cam_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, telemetry->measured.position[11], telemetry->measured.position[10]);
    publishTfFrame("/pt", cam_pose, *br);
    publish_cam_frames();
}

void callback2(const geometry_msgs::PoseStamped::ConstPtr &tcp)
{
    // sensor_msgs::JointState joint_state;
    // joint_state.header = telemetry->header;

    // for (int i = 2; i < telemetry->joint_names.size() - 3; ++i)
    // {
    //     joint_state.name.push_back(telemetry->joint_names[i]);
    //     joint_state.position.push_back(telemetry->measured.position[i]);
    //     joint_state.velocity.push_back(telemetry->commanded.velocity[i]);
    //     joint_state.effort.push_back(telemetry->commanded.acceleration[i]);

    // }
    // joint_state.name.push_back("joint_before_finger2");
    // joint_state.position.push_back(telemetry->measured.position[9] / 2);
    // joint_state.velocity.push_back(telemetry->commanded.velocity[9]);
    // joint_state.effort.push_back(telemetry->commanded.acceleration[9]);

    // joint_state.name.push_back("joint_before_finger1");
    // joint_state.position.push_back(-telemetry->measured.position[9] / 2);
    // joint_state.velocity.push_back(telemetry->commanded.velocity[9]);
    // joint_state.effort.push_back(telemetry->commanded.acceleration[9]);

    // joint_state_pub.publish(joint_state);

    // geometry_msgs::PoseStamped cam_pose;
    // cam_pose.header.frame_id = "/pt_base";
    // cam_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, telemetry->measured.position[11], telemetry->measured.position[10]);
    // ROS_INFO_STREAM("???");
    publishTfFrame("/tcp", *tcp, *br2);
    // publish_cam_frames();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "telemetry_to_joint_state");
    ros::NodeHandle n;
    br = new tf::TransformBroadcaster();
    br2 = new tf::TransformBroadcaster();

    joint_state_pub = n.advertise < sensor_msgs::JointState > ( "/joint_states", 10 );

    ros::Subscriber js_sub = n.subscribe("/euroc_interface_node/telemetry", 1000, callback);
    ros::Subscriber lwr_tool = n.subscribe("/euroc_interface_node/lwr_base_to_tcp", 1000, callback2);

    // ros::AsyncSpinner spinner(2); // Use 4 threads
    // spinner.start();
    ros::spin();
    return 0;
}












