/**
* This class implements the action server to move an selected arm.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>


using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> Server;
ros::ServiceClient move_along_joint_path_client;

void set_gripper_pos(const control_msgs::GripperCommandGoalConstPtr &goal, ros::NodeHandle *nh, Server *server_arm)
{
    euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv;
    move_along_joint_path_srv.request.joint_names.push_back("gripper");

    // const unsigned int nr_lwr_joints = 7;

    // move_along_joint_path_srv.request.joint_names = lwr_joints; // Select all lwr joints


    euroc_c2_msgs::Configuration configuration;
    configuration.q.push_back(2*goal->command.position);
    move_along_joint_path_srv.request.path.push_back(configuration);



    move_along_joint_path_srv.request.joint_limits.resize(1);

    euroc_c2_msgs::Limits &limits = move_along_joint_path_srv.request.joint_limits[0];
    limits.max_velocity = 0.5; 
    limits.max_acceleration = 0.5;


    move_along_joint_path_client.call(move_along_joint_path_srv);
    std::string &move_error_message = move_along_joint_path_srv.response.error_message;
    if (!move_error_message.empty())
    {
        std::cout << "Move failed: " + move_error_message << std::endl;
        server_arm->setAborted();
    }
    server_arm->setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_controller");
    ros::NodeHandle n;

    const std::string euroc_c2_interface = "/euroc_interface_node";
    const std::string move_along_joint_path = euroc_c2_interface + "/move_along_joint_path";
    ROS_INFO_STREAM("Waiting for Service: " << move_along_joint_path);
    ros::service::waitForService(move_along_joint_path);
    move_along_joint_path_client = n.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path);

    Server server_arm(n, "/gripper_controller/gripper_action", boost::bind(&set_gripper_pos, _1, &n, &server_arm), false);

    server_arm.start();

    ROS_INFO("Ready to move the arms!.");
    ros::spin();
    return 0;
}