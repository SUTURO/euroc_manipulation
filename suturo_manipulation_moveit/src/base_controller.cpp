/**
* This class implements the action server to move the base.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>


using namespace std;

typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> Server;
ros::ServiceClient move_along_joint_path_client;

void goal_call_back(Server::GoalHandle gh)
{
    gh.setAccepted();
    ROS_INFO_STREAM("Got base request.");

    control_msgs::FollowJointTrajectoryGoal goal = *gh.getGoal();
    euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv;
    move_along_joint_path_srv.request.joint_names = goal.trajectory.joint_names;

    const unsigned int nr_lwr_joints = 2;

    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator p = goal.trajectory.points.begin(); p != goal.trajectory.points.end(); ++p)
    {
        euroc_c2_msgs::Configuration configuration;
        configuration.q = p->positions;
        move_along_joint_path_srv.request.path.push_back(configuration);
    }

    // ROS_INFO_STREAM(move_along_joint_path_srv.request);
    move_along_joint_path_srv.request.joint_limits.resize(nr_lwr_joints);
    for (unsigned int i = 0; i < nr_lwr_joints; ++i)
    {
        euroc_c2_msgs::Limits &limits = move_along_joint_path_srv.request.joint_limits[i];
        limits.max_velocity = 0.165;
        limits.max_acceleration = 4;
    }

    move_along_joint_path_client.call(move_along_joint_path_srv);
    std::string &move_error_message = move_along_joint_path_srv.response.error_message;
    std::string &move_stop_message = move_along_joint_path_srv.response.stop_reason;
    ROS_INFO_STREAM("stop reason: " + move_stop_message);
    if (!move_error_message.empty())
    {
        ROS_ERROR_STREAM("Move failed: " + move_error_message);
        gh.setAborted();
    }
    ROS_INFO_STREAM("Error Message: " + move_error_message);    
    gh.setSucceeded();
}

void cancel_call_back(Server::GoalHandle gh)
{
    ROS_ERROR_STREAM("cancel");
    gh.setCanceled();
    ROS_ERROR_STREAM(gh.getGoalStatus());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;

    const std::string euroc_c2_interface = "/euroc_interface_node";
    const std::string move_along_joint_path = euroc_c2_interface + "/move_along_joint_path";
    ROS_INFO_STREAM("Waiting for Service: " << move_along_joint_path);
    ros::service::waitForService(move_along_joint_path);
    move_along_joint_path_client = n.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path);

    Server base_server(n, "/base_controller/follow_joint_trajectory", false   );

    base_server.registerCancelCallback(&cancel_call_back);
    base_server.registerGoalCallback(&goal_call_back);

    base_server.start();

    ROS_INFO("Ready to move the base!.");
    ros::spin();
    return 0;
}
