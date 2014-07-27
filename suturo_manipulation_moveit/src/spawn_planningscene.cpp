#include <ros/ros.h>

// MoveIt!
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

using namespace std;

moveit_msgs::CollisionObject make_box(string name, double posx, double posy, double posz, double dx, double dy, double dz, double r, double p, double y)
{
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "/odom_combined";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitive_poses.resize(1);

  co.id = name;

  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dx;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dy;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = dz;
  co.primitive_poses[0].position.x = posx;
  co.primitive_poses[0].position.y = posy;
  co.primitive_poses[0].position.z = posz;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
  return co;
}

moveit_msgs::CollisionObject make_cylinder(string name, double posx, double posy, double posz, double dh, double dr, double r, double p, double y)
{
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "/odom_combined";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  co.primitive_poses.resize(1);

  co.id = name;

  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = dh;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = dr;
  co.primitive_poses[0].position.x = posx;
  co.primitive_poses[0].position.y = posy;
  co.primitive_poses[0].position.z = posz;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
  return co;
}

void spawn_task1_v1(ros::Publisher pub_co)
{

	
  ros::WallDuration(1.0).sleep();

  moveit_msgs::CollisionObject co;
  //add box
  co = make_box("red_cube", -0.3, -0.4, 0.03, 0.05, 0.05, 0.05, 0, 0, 0);

  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;  
  pub_co.publish(co);
  
  //add Cylinder
  co = make_cylinder("green_cylinder", -0.5, 0.1, 0.051, 0.1, 0.02, -3.1415, 0, 0.8);

  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;  
  pub_co.publish(co);


  //   co.id = "blue_handle";
  // co.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_co.publish(co);

  // // add table
  // co.operation = moveit_msgs::CollisionObject::ADD;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.75;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.8;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tischposiZ;
  // co.primitive_poses[0].position.x = 0.85;
  // co.primitive_poses[0].position.y = 0;
  // co.primitive_poses[0].position.z = tischposiZ/2;
  // co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);  
  // pub_co.publish(co);

  ros::WallDuration(2.0).sleep();
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "right_arm_pick_place");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	spawn_task1_v1(pub_co);
  	
	ROS_INFO_STREAM("finish");
	ros::waitForShutdown();
	return 0;
}
