/**
* This class implements the action server to move an selected arm.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/EnableServoMode.h>
#include <euroc_c2_msgs/Telemetry.h>
#include <euroc_c2_msgs/SetServoTarget.h>


using namespace std;

typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> Server;
ros::ServiceClient move_along_joint_path_client;
ros::ServiceClient enable_servo_client;
ros::ServiceClient set_servo_target_client;

void enable_servo_mode(bool muh)
{
    euroc_c2_msgs::EnableServoMode enable;
    enable.request.servo_mode_active = muh;
    enable_servo_client.call(enable);
}

// void goal_call_back(Server::GoalHandle gh)
// {
//     // ROS_ERROR_STREAM("GOAL");
//     gh.setAccepted();

//     control_msgs::FollowJointTrajectoryGoal goal = *gh.getGoal();
//     euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv;
//     move_along_joint_path_srv.request.joint_names = goal.trajectory.joint_names;

//     const unsigned int nr_lwr_joints = 7;

//     move_along_joint_path_srv.request.joint_limits.resize(nr_lwr_joints);
//     for (unsigned int i = 0; i < nr_lwr_joints; ++i)
//     {
//         euroc_c2_msgs::Limits &limits = move_along_joint_path_srv.request.joint_limits[i];
//         limits.max_velocity = 40 * M_PI / 180.0; // 20 degrees per second
//         limits.max_acceleration = 400 * M_PI / 180.0;
//     }

//     std::string move_error_message;

//     move_along_joint_path_srv.request.path.resize(1);

//     actionlib_msgs::GoalStatus muh = gh.getGoalStatus();

//     for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator p = goal.trajectory.points.begin(); p != goal.trajectory.points.end() ; ++p)
//     {
//         euroc_c2_msgs::Configuration configuration;
//         configuration.q = p->positions;
//         move_along_joint_path_srv.request.path[0] = configuration;

//         move_along_joint_path_client.call(move_along_joint_path_srv);

//         move_error_message = move_along_joint_path_srv.response.error_message;

//         muh = gh.getGoalStatus();
//         if (muh.status == actionlib_msgs::GoalStatus::PREEMPTED ||
//                 muh.status == actionlib_msgs::GoalStatus::RECALLED)
//             break;
//     }


//     muh = gh.getGoalStatus();
//     if (muh.status == actionlib_msgs::GoalStatus::PREEMPTED ||
//             muh.status == actionlib_msgs::GoalStatus::RECALLED)
//         return;
//     if (!move_error_message.empty())
//     {
//         std::cout << "Move failed: " + move_error_message << std::endl;
//         gh.setAborted();
//     }
//     gh.setSucceeded();
// }

void goal_call_back(Server::GoalHandle gh)
{
    gh.setAccepted();
    // enable_servo_mode(true);

    //    ros::WallDuration(1).sleep();

    control_msgs::FollowJointTrajectoryGoal goal = *gh.getGoal();
    euroc_c2_msgs::SetServoTarget servo_target;
    servo_target.request.joint_names = goal.trajectory.joint_names;

    const unsigned int nr_lwr_joints = 7;

    std::string move_error_message;

    double i = 0;

    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator p = goal.trajectory.points.begin(); p != goal.trajectory.points.end(); ++p)
    {
        // std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator p = goal.trajectory.points.end()-1;
        euroc_c2_msgs::Configuration configuration;
        configuration.q = p->positions;
        servo_target.request.target = configuration;
        // ROS_INFO_STREAM("Call service");
        set_servo_target_client.call(servo_target);

        move_error_message = servo_target.response.error_message;

        actionlib_msgs::GoalStatus muh = gh.getGoalStatus();
        // ROS_INFO_STREAM(gh.getGoalStatus());
        if (muh.status == actionlib_msgs::GoalStatus::PREEMPTED ||
                muh.status == actionlib_msgs::GoalStatus::RECALLED)
            break;

        // i += 0.05;
        // ros::WallDuration(1).sleep();
    }

    // move_along_joint_path_client.call(servo_target);
    // std::string &move_error_message = servo_target.response.error_message;
    // ROS_INFO_STREAM(gh.getGoalStatus());
    // actionlib_msgs::GoalStatus muh = gh.getGoalStatus();
    // if (muh.status == actionlib_msgs::GoalStatus::PREEMPTED ||
    //         muh.status == actionlib_msgs::GoalStatus::RECALLED)
    //     return;
    // enable_servo_mode(false);
    if (!move_error_message.empty())
    {
        ROS_ERROR_STREAM("Move failed: " << move_error_message);
        gh.setAborted();
    }
    gh.setSucceeded();
}

void cancel_call_back(Server::GoalHandle gh)
{
    ROS_ERROR_STREAM("cancel");
    gh.setCanceled();
    // ROS_ERROR_STREAM(gh.getGoalStatus());
}

void callback(const euroc_c2_msgs::Telemetry::ConstPtr &telemetry)
{
    for (int i = 2; i < telemetry->joint_names.size() - 3; ++i)
    {
        if (telemetry->measured.torque[i] > 10)
        {
            ROS_WARN_STREAM(telemetry->joint_names[i] << "  " << telemetry->measured.torque[i] << "   " << telemetry->measured.external_torque[i]);
            cout << endl;
        }

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;

    const std::string euroc_c2_interface = "/euroc_interface_node";
    const std::string move_along_joint_path = euroc_c2_interface + "/move_along_joint_path";

    const std::string enable = euroc_c2_interface + "/enable_servo_mode";
    const std::string servo = euroc_c2_interface + "/set_servo_target";


    ROS_INFO_STREAM("Waiting for Service: " << move_along_joint_path);
    // ros::service::waitForService(move_along_joint_path);
    // move_along_joint_path_client = n.serviceClient<euroc_c2_msgs::MoveAlongJointPath>(move_along_joint_path);

    ros::service::waitForService(enable);
    enable_servo_client = n.serviceClient<euroc_c2_msgs::EnableServoMode>(enable);
    // enable_servo_mode(true);

    ros::service::waitForService(servo);
    set_servo_target_client = n.serviceClient<euroc_c2_msgs::SetServoTarget>(servo);

    // ros::Subscriber js_sub = n.subscribe("/euroc_interface_node/telemetry", 1000, callback);

    // Server server_arm(n, "/arm_controller/follow_joint_trajectory", boost::bind(&follow_joint_trajectory, _1, &n, &server_arm), false);
    // Server arm_server(n, "/arm_controller/follow_joint_trajectory", false   );
    // ros::Subscriber js_sub = n.subscribe("/arm_controller/follow_joint_trajectory/cancel", 1000, callback);
    Server arm_server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&goal_call_back, _1), boost::bind(&cancel_call_back, _1), false   );

    // arm_server.registerCancelCallback(&cancel_call_back);
    // arm_server.registerGoalCallback(&goal_call_back);
    // boost::bind(&goal_call_back, _1),
    // boost::bind(&cancel_call_back, _1),
    // false);


    arm_server.start();

    ROS_INFO("Ready to move the arms!.");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
