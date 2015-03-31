#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>
#include <suturo_perception_msgs/GetCameraPerception.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_to_planningscene");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle n;

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetCameraPerception>("/suturo/GetGripper");
  ros::Publisher pub_co = n.advertise<moveit_msgs::CollisionObject>("collision_object", 20);
  suturo_perception_msgs::GetCameraPerception gripperSrv;
  gripperSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  // run until service gets shut down
  while (true)
  {
    if (clusterClient.call(gripperSrv))
    {
      for (int i = 0; i < gripperSrv.response.objects.size(); i++)
      {
        suturo_perception_msgs::EurocObject obj = gripperSrv.response.objects.at(i);
        if (!obj.mpe_success)
        {
          ROS_INFO_STREAM("MPE unsuccessful");
          continue;
        }
        ROS_INFO_STREAM("MPE SUCCESS");
        // Uncomment this if not set in msg, these params are needed!
//        obj.mpe_object.id = "test";
//        obj.mpe_object.header.stamp = ros::Time::now();
//        obj.mpe_object.header.frame_id = "/tdepth_pcl";
//        obj.mpe_object.operation = moveit_msgs::CollisionObject::ADD;
        pub_co.publish(obj.mpe_object);
      }
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetGripper");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }
}
