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
    double height, radius;
    Pose pose;
  };
  struct Box
  {
    double width, height, depth;
    Pose pose;
  };
  SpawnPlanningscene(ros::Publisher*);
  void publishObjects();
  void publishObstacles();
  void loadYaml(string);

private:
  moveit_msgs::CollisionObject make_handle(string, Pose);
  void extractRelevantNodes(YAML::Node&);
  void publishObject(string, const YAML::Node&, const YAML::Node&);
  void addBox(moveit_msgs::CollisionObject&, Box);
  void addCylinder(moveit_msgs::CollisionObject&, Cylinder);
  void addPose(moveit_msgs::CollisionObject&, Pose, Pose);
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
  node["size"][0] >> box.height;
  node["size"][1] >> box.width;
  node["size"][2] >> box.depth;
  node["pose"] >> box.pose;
}

void operator >>(const YAML::Node& node, SpawnPlanningscene::Cylinder& box)
{
  node["radius"] >> box.radius;
  node["length"] >> box.height;
  node["pose"] >> box.pose;
}

SpawnPlanningscene::SpawnPlanningscene(ros::Publisher* pub_co)
{
  this->pub_co = pub_co;
}

void SpawnPlanningscene::publishObjects()
{
  if (publicDescription.get() != NULL && internalDescription.get() != NULL)
  {
    const YAML::Node& objects = (*publicDescription);
    for (YAML::Iterator i = objects.begin(); i != objects.end(); ++i)
    {
      std::string name;
      i.first() >> name;
      if (!(*internalDescription).FindValue(name))
      {
        continue;
      }
      publishObject(name, i.second(), (*internalDescription)[name.c_str()]);

    }
  }
}

void SpawnPlanningscene::publishObject(string name, const YAML::Node& publicObject, const YAML::Node& internalObject)
{
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "/odom_combined";
  co.id = name;
  const YAML::Node& shape = publicObject["shape"];
  for (YAML::Iterator i = shape.begin(); i != shape.end(); ++i)
  {
    string type;
    (*i)["type"] >> type;
    Pose pose;
    internalObject["start_pose"] >> pose;
    if (type == "box")
    {
      Box box;
      (*i) >> box;
      addBox(co, box);
      addPose(co, box.pose, pose);
    }
    else if (type == "cylinder")
    {
      Cylinder cylinder;
      (*i) >> cylinder;
      addCylinder(co, cylinder);
      addPose(co, cylinder.pose, pose);
    }
  }
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co->publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  pub_co->publish(co);
}

void SpawnPlanningscene::addBox(moveit_msgs::CollisionObject& co, Box box)
{
  shape_msgs::SolidPrimitive moveitPrimitive;
  moveitPrimitive.type = shape_msgs::SolidPrimitive::BOX;
  moveitPrimitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  moveitPrimitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = box.height;
  moveitPrimitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = box.width;
  moveitPrimitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = box.depth;
  co.primitives.push_back(moveitPrimitive);
}

void SpawnPlanningscene::addCylinder(moveit_msgs::CollisionObject& co, Cylinder cylinder)
{
  shape_msgs::SolidPrimitive moveitPrimitive;
  moveitPrimitive.type = shape_msgs::SolidPrimitive::CYLINDER;
  moveitPrimitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  moveitPrimitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = cylinder.height;
  moveitPrimitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = cylinder.radius;
  co.primitives.push_back(moveitPrimitive);
}

void SpawnPlanningscene::addPose(moveit_msgs::CollisionObject& co, Pose relativePose, Pose absoultePose)
{
  geometry_msgs::Pose moveitPose;
  tf::Vector3 offset(relativePose.x, relativePose.y, relativePose.z);
  tf::Quaternion quat;
  quat.setRPY(absoultePose.roll, absoultePose.pitch, absoultePose.yaw);
  tf::Vector3 rotatedOffset = tf::quatRotate(quat, offset);
  moveitPose.position.x = absoultePose.x + rotatedOffset.x();
  moveitPose.position.y = absoultePose.y + rotatedOffset.y();
  moveitPose.position.z = absoultePose.z + rotatedOffset.z();
  tf::quaternionTFToMsg(quat, moveitPose.orientation);
  co.primitive_poses.push_back(moveitPose);
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
      moveit_msgs::CollisionObject co;
      co.header.stamp = ros::Time::now();
      co.header.frame_id = "/odom_combined";
      co.id = name;
      Box box;
      Pose pose;
      i.second()["shape"][0] >> box;
      i.second()["start_pose"] >> pose;
      addBox(co, box);
      addPose(co, box.pose, pose);
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      pub_co->publish(co);
      co.operation = moveit_msgs::CollisionObject::ADD;
      pub_co->publish(co);
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
  extractRelevantNodes(doc);
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

void SpawnPlanningscene::extractRelevantNodes(YAML::Node& doc)
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
  sps.publishObjects();
  sps.publishObstacles();
  ros::WallDuration(0.5).sleep();

  ROS_INFO_STREAM("finish");
  return 0;
}
