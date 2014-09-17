#include <ros/ros.h>

// MoveIt!
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <fstream>
#include <yaml-cpp/yaml.h>


static const std::string ROBOT_DESCRIPTION = "robot_description";

using namespace std;

class SpawnPlanningscene
{
public:
  struct Pose
  {
    double x, y, z, roll, pitch, yaw;
  };
  struct Cylinder
  {
      double height,radius;
  };
  struct Box
  {
      double width,height,depth;
  };
  enum PublishType
  {
    BOX, PLANE, CYLINDER, HANDLE, OBSTACLE
  };
  SpawnPlanningscene(ros::Publisher* pub_co);
  void publish(PublishType type, string name);
  void publishObstacles();
  void spawnPlane();
  void loadYaml(string yamlfile);

private:
  moveit_msgs::CollisionObject make_box(string name, Pose pose, Box dim);
  moveit_msgs::CollisionObject make_plane(string name);
  moveit_msgs::CollisionObject make_cylinder(string name, Pose pose, Cylinder dim);
  moveit_msgs::CollisionObject make_handle(string name, Pose pose);
  void extractRelevantValues(YAML::Node &doc);
  std::auto_ptr<YAML::Node> publicDescription;
  std::auto_ptr<YAML::Node> internalDescription;
  std::auto_ptr<YAML::Node> obstaclesInternal;
  ros::Publisher* pub_co;

};

void operator >>(const YAML::Node& node, SpawnPlanningscene::Pose& p)
{
  node[0] >> p.x;
  node[1] >> p.y;
  node[2] >> p.z;
  node[3] >> p.roll;
  node[4] >> p.pitch;
  node[5] >> p.yaw;
}

void operator >>(const YAML::Node& node, SpawnPlanningscene::Box& box)
{
  const YAML::Node& s = node["shape"][0]["size"];
  s[0] >> box.height;
  s[1] >> box.width;
  s[2] >> box.depth;
}


void operator >>(const YAML::Node& node, SpawnPlanningscene::Cylinder& box)
{
  node["shape"][0]["radius"] >> box.radius;
  node["shape"][0]["length"] >> box.height;
}

SpawnPlanningscene::SpawnPlanningscene(ros::Publisher* pub_co)
{
  this->pub_co = pub_co;
}

moveit_msgs::CollisionObject SpawnPlanningscene::make_box(string name, Pose pose, Box dim)
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

    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = dim.height;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dim.width;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dim.depth;
    co.primitive_poses[0].position.x = pose.x;
    co.primitive_poses[0].position.y = pose.y;
    co.primitive_poses[0].position.z = pose.z;
    co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(pose.roll, pose.pitch, pose.yaw);
    return co;
}

moveit_msgs::CollisionObject SpawnPlanningscene::make_plane(string name)
{
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "/odom_combined";
    co.operation = moveit_msgs::CollisionObject::ADD;

    co.id = name;

    geometry_msgs::Pose p;
    p.position.x = 0;
    p.position.y = 0;
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1;

    shape_msgs::Plane plane;
    plane.coef[0] = 0;
    plane.coef[1] = 0;
    plane.coef[2] = 1;
    plane.coef[3] = 0;

    co.planes.push_back(plane);
    co.plane_poses.push_back(p);
    ROS_INFO_STREAM(co);
    return co;
}

moveit_msgs::CollisionObject SpawnPlanningscene::make_cylinder(string name, Pose pose, Cylinder dim)
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

    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = dim.height;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = dim.radius;
    co.primitive_poses[0].position.x = pose.x;
    co.primitive_poses[0].position.y = pose.y;
    co.primitive_poses[0].position.z = pose.z;
    co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(pose.roll, pose.pitch, pose.yaw);
    return co;
}

moveit_msgs::CollisionObject SpawnPlanningscene::make_handle(string name, Pose pose)
{
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "/odom_combined";
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(3);
    co.primitive_poses.resize(3);

    co.id = name;

    co.primitives[1].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[1].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
    co.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
    co.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
    co.primitive_poses[1].position.x = pose.x;//0.0681482099;//0.16118567395 posx + 0;
    co.primitive_poses[1].position.y = pose.y;//-0.16118567395;// posy + 0.35;
    co.primitive_poses[1].position.z = pose.z;//;//0.16118567395 posz + 0;
    co.primitive_poses[1].orientation = tf::createQuaternionMsgFromRollPitchYaw(pose.roll, pose.pitch, pose.yaw);

    co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.3;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.01;
    co.primitive_poses[0].position.x = co.primitive_poses[1].position.x + -0.0681482099;// + 0;
    co.primitive_poses[0].position.y = co.primitive_poses[1].position.y + 0.16118567395;// + 0.175;
    co.primitive_poses[0].position.z = co.primitive_poses[1].position.z + 0;
    co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(pose.roll, pose.pitch, pose.yaw);

    co.primitives[2].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[2].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[2].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
    co.primitives[2].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
    co.primitives[2].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
    co.primitive_poses[2].position.x = co.primitive_poses[0].position.x + -0.0681482099;//0.16118567395 posx + 0;
    co.primitive_poses[2].position.y = co.primitive_poses[0].position.y + 0.16118567395;// posy + 0.35;
    co.primitive_poses[2].position.z = co.primitive_poses[0].position.z + 0;//0.16118567395 posz + 0;
    co.primitive_poses[2].orientation = tf::createQuaternionMsgFromRollPitchYaw(pose.roll, pose.pitch, pose.yaw);
    return co;
}

void SpawnPlanningscene::publish(PublishType type, string name)
{
  moveit_msgs::CollisionObject co;
  Pose pose;
  Box box;
  Cylinder cylinder;
  switch (type)
  {
    case BOX:
      (*publicDescription)[name.c_str()] >> box;
      (*internalDescription)[name.c_str()]["start_pose"] >> pose;
      co = make_box(name, pose, box);
      break;
    case PLANE:
      co = make_plane(name);
      break;
    case CYLINDER:
      (*publicDescription)[name.c_str()] >> cylinder;
      (*internalDescription)[name.c_str()]["start_pose"] >> pose;
      co = make_cylinder(name, pose, cylinder);
      break;
    case HANDLE:
      (*internalDescription)[name.c_str()]["start_pose"] >> pose;
      co = make_handle(name, pose);
      break;
    case OBSTACLE:
      (*obstaclesInternal)[name.c_str()] >> box;
      (*obstaclesInternal)[name.c_str()]["start_pose"] >> pose;
      co = make_box(name, pose, box);
      break;

  }
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co->publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  pub_co->publish(co);
}

void SpawnPlanningscene::publishObstacles()
{
  if (obstaclesInternal.get() != NULL)
  {
    const YAML::Node& obstacles = (*obstaclesInternal);
    for (YAML::Iterator i = obstacles.begin(); i != obstacles.end(); ++i)
    {
      std::string name;
      i.first() >> name;
      publish(SpawnPlanningscene::OBSTACLE, name);
    }
  }
}

void SpawnPlanningscene::loadYaml(string yamlfile)
{
  string includePath = "/opt/euroc_c2s1/scenes/" + yamlfile;
  ifstream filestream(includePath.c_str());
  YAML::Parser parser(filestream);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  extractRelevantValues(doc);
  if (doc.FindValue("includes"))
  {
    const YAML::Node& includes = doc["includes"];
    for (YAML::Iterator i = includes.begin(); i != includes.end(); ++i)
    {
      std::string includeName;
      *i >> includeName;
      loadYaml(includeName);
    }
  }
  filestream.close();
}

void SpawnPlanningscene::extractRelevantValues(YAML::Node& doc)
{
  if (publicDescription.get() == NULL && doc.FindValue("public_description")
      && doc.FindValue("public_description")->FindValue("objects"))
  {
    publicDescription = doc["public_description"]["objects"].Clone();
  }
  if (doc.FindValue("internal_description"))
  {
    if (internalDescription.get() == NULL && doc.FindValue("internal_description")->FindValue("objects"))
    {
      internalDescription = doc["internal_description"]["objects"].Clone();
    }
    if (obstaclesInternal.get() == NULL && doc.FindValue("internal_description")->FindValue("obstacles"))
    {
      obstaclesInternal = doc["internal_description"]["obstacles"].Clone();
    }
  }
}

void SpawnPlanningscene::spawnPlane()
{
//  publish(SpawnPlanningscene::PLANE, "plane");

// or

//  Pose p = {0, 0, -0.005, 0, 0, 0};
//  Box b = {2, 2, 0};
//  moveit_msgs::CollisionObject co = make_box("ground", p, b);
//  co.operation = moveit_msgs::CollisionObject::REMOVE;
//  pub_co->publish(co);
//
//  co.operation = moveit_msgs::CollisionObject::ADD;
//  pub_co->publish(co);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "spawn_planningscene");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 20);
  SpawnPlanningscene sps(&pub_co);
  string map;
  if (argc > 1)
  {
    map = string(argv[1]);
  }
  else
  {
    map = string("task1_v1");
  }
  sps.loadYaml(map + ".yml");
  ros::WallDuration(0.5).sleep();
  sps.publish(SpawnPlanningscene::BOX, "red_cube");
  sps.publish(SpawnPlanningscene::CYLINDER, "green_cylinder");
  sps.publish(SpawnPlanningscene::HANDLE, "blue_handle");
  sps.publishObstacles();
  ros::WallDuration(0.5).sleep();

  ROS_INFO_STREAM("finish");
  // ros::waitForShutdown();
  return 0;
}
