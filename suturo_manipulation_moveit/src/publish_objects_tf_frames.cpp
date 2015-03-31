#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/capability_names.h>

#include <suturo_manipulation_moveit/node_status.hpp>

/**
 * This Programm publishes a tf frame into every collisionobject and attached object.
 */

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

geometry_msgs::PoseStamped getCamPose(ros::NodeHandle n)
{
    geometry_msgs::PoseStamped cam_pose;

    //set orientation
    cam_pose.pose.orientation.w = 1;

    if (!n.getParam("/suturo_manipulation_tf_publisher/cam_frame", cam_pose.header.frame_id))
    {
        ROS_ERROR_STREAM("Failed to Frame for Cam.");
    }
    if (!n.getParam("/suturo_manipulation_tf_publisher/cam_x", cam_pose.pose.position.x))
    {
        ROS_ERROR_STREAM("Failed to get x coordinate for Cam.");
    }
    if (!n.getParam("/suturo_manipulation_tf_publisher/cam_y", cam_pose.pose.position.y))
    {
        ROS_ERROR_STREAM("Failed to get y coordinate for Cam.");
    }
    if (!n.getParam("/suturo_manipulation_tf_publisher/cam_z", cam_pose.pose.position.z))
    {
        ROS_ERROR_STREAM("Failed to get z coordinate for Cam.");
    }

    return cam_pose;
}

int getPlanningScene(ros::ServiceClient &ps_client, moveit_msgs::PlanningScene &ps)
{
    //create msg to get Objectnames and Objectgeometry from planningscene
    moveit_msgs::GetPlanningScene msg;
    msg.request.components.components = 1023;

    //get planningscene

    ps_client.call(msg);
    if (ps_client.call(msg))
    {
        ps = msg.response.scene;
    }
    else
    {
        ROS_ERROR("Failed to call service to get planningscene.");
        return 0;
    }
    return 1;
}

int getObjects(ros::ServiceClient &ps_client, std::vector<moveit_msgs::CollisionObject> &cos)
{
    moveit_msgs::PlanningScene ps;
    if (!getPlanningScene(ps_client, ps))
    {
        return 0;
    }
    cos = ps.world.collision_objects;
    return 1;
}

int getAttachedObjects(ros::ServiceClient &ps_client, std::vector<moveit_msgs::AttachedCollisionObject> &cos)
{
    moveit_msgs::PlanningScene ps;
    if (!getPlanningScene(ps_client, ps))
    {
        return 0;
    }
    cos = ps.robot_state.attached_collision_objects;
    return 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_objects_tf_frames");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;

    std::vector<moveit_msgs::CollisionObject> cos;
    std::vector<moveit_msgs::AttachedCollisionObject> acos;

    // boost::this_thread::sleep(boost::posix_time::seconds(3));

    // geometry_msgs::PoseStamped cam_pose = getCamPose(n);

    geometry_msgs::PoseStamped temp_pose;
    ROS_INFO_STREAM("tf publisher started..........");

    ros::service::waitForService(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    ros::ServiceClient ps_service_client = n.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);

    ros::WallDuration(1.0).sleep();
    suturo_manipulation::NodeStatus node_status(n);
    node_status.nodeStarted(suturo_startup_msgs::ManipulationNodeStatus::NODE_PUBLISH_OBJECT_FRAMES);

    while (getObjects(ps_service_client, cos) && getAttachedObjects(ps_service_client, acos))
    {
        //publish tf frame in every collisionobject

        // if ()
        // {
            // ROS_INFO_STREAM(cos.size());
            for (std::vector<moveit_msgs::CollisionObject>::iterator co = cos.begin(); co != cos.end(); ++co)
            {
                if (co->primitive_poses.size() > 0)
                {
                    temp_pose.pose = co->primitive_poses[0];
                    temp_pose.header = co->header;
                    publishTfFrame(co->id, temp_pose, br);
                }
            }
        // }

        // if (getAttachedObjects(ps_service_client, acos))
        // {
            for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator co = acos.begin(); co != acos.end(); ++co)
            {
                if (co->object.primitive_poses.size() > 0)
                {
                    temp_pose.pose = co->object.primitive_poses[0];
                    temp_pose.header = co->object.header;
                    publishTfFrame(co->object.id, temp_pose, br);
                }
            }
        // }

        //publish tf frame in every attachedobject
        // for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator aco = acos.begin(); aco != acos.end(); ++aco)
        // {
        //     if (aco->object.primitive_poses.size() >= 1)
        //     {

        //         ROS_DEBUG_STREAM(*aco);
        //         temp_pose.pose = aco->object.primitive_poses[0];
        //         temp_pose.header = aco->object.header;
        //         publishTfFrame(aco->object.id, temp_pose, transform, br);
        //     }
        //     else
        //     {
        //         ROS_DEBUG_STREAM(*aco);
        //         temp_pose.pose = aco->object.mesh_poses[0];
        //         temp_pose.header = aco->object.header;
        //         publishTfFrame(aco->object.id, temp_pose, transform, br);
        //     }
        // }

        //publish cam_frame
        // publishTfFrame("webcam", cam_pose, transform, br);

        ros::WallDuration(0.1).sleep();
    }

    return 0;
}


















