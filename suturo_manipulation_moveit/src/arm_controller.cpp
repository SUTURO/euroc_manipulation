/**
* This class implements the action server to move an selected arm.
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
    ROS_ERROR_STREAM("GOAL");
    // ros::WallDuration(20.0).sleep();
    gh.setAccepted();
    control_msgs::FollowJointTrajectoryGoal goal = *gh.getGoal();
    euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv;
    move_along_joint_path_srv.request.joint_names = goal.trajectory.joint_names;

    const unsigned int nr_lwr_joints = 7;

    // move_along_joint_path_srv.request.joint_names = lwr_joints; // Select all lwr joints

    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator p = goal.trajectory.points.begin(); p != goal.trajectory.points.end(); ++p)
    {
        euroc_c2_msgs::Configuration configuration;
        configuration.q = p->positions;
        move_along_joint_path_srv.request.path.push_back(configuration);
    }


    move_along_joint_path_srv.request.joint_limits.resize(nr_lwr_joints);
    for (unsigned int i = 0; i < nr_lwr_joints; ++i)
    {
        euroc_c2_msgs::Limits &limits = move_along_joint_path_srv.request.joint_limits[i];
        limits.max_velocity = 20 * M_PI / 180.0; // 20 degrees per second
        limits.max_acceleration = 400 * M_PI / 180.0;
    }

    move_along_joint_path_client.call(move_along_joint_path_srv);
    std::string &move_error_message = move_along_joint_path_srv.response.error_message;
    // actionlib_msgs::GoalStatus muh = gh.getGoalStatus();
    // if (muh.status == actionlib_msgs::GoalStatus::PREEMPTED ||
    //     muh.status == actionlib_msgs::GoalStatus::RECALLED)
    //     return;
    if (!move_error_message.empty())
    {
        std::cout << "Move failed: " + move_error_message << std::endl;
        gh.setAborted();
    }
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
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;

    const std::string euroc_c2_interface = "/euroc_interface_node";
    const std::string move_along_joint_path = euroc_c2_interface + "/move_along_joint_path";
    ROS_INFO_STREAM("Waiting for Service: " << move_along_joint_path);
    ros::service::waitForService(move_along_joint_path);
    move_along_joint_path_client = n.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path);

    // Server server_arm(n, "/arm_controller/follow_joint_trajectory", boost::bind(&follow_joint_trajectory, _1, &n, &server_arm), false);
    Server arm_server(n, "/arm_controller/follow_joint_trajectory", false   );

    arm_server.registerCancelCallback(&cancel_call_back);
    arm_server.registerGoalCallback(&goal_call_back);
        // boost::bind(&goal_call_back, _1),
        // boost::bind(&cancel_call_back, _1),
        // false);

    arm_server.start();

    ROS_INFO("Ready to move the arms!.");
    ros::spin();
    return 0;
}
