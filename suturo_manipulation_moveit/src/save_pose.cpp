#include <ros/ros.h>

#include <tinyxml.h>
#include <boost/filesystem.hpp>
// Joint State Message
#include <sensor_msgs/JointState.h>

using namespace std;

const char* kukaFile;

void writeJoints(TiXmlElement* groupStateElement, map<string, double>& joints)
{
  for (map<string, double>::iterator iter = joints.begin(); iter != joints.end(); ++iter)
  {
    TiXmlElement * joint = new TiXmlElement("joint");
    groupStateElement->LinkEndChild(joint);
    joint->SetAttribute("name", iter->first);
    char value[30];
    sprintf(value, "%.20f", iter->second);
    joint->SetAttribute("value", value);
  }
}

void writeSRDF(const string filename, const string poseName, map<string, double>& joints)
{
  TiXmlDocument doc(filename);
  if (!doc.LoadFile())
  {
    printf("File could not be loaded.\n");
    return;
  }

  TiXmlHandle hDoc(&doc);
  TiXmlElement* robot = hDoc.FirstChildElement().Element();
  for (;; robot = robot->NextSiblingElement())
  {
    if (!robot)
    {
      printf("File does not contain a robot root element.\n");
      return;
    }
    const char *rName = robot->Attribute("name");
    // TODO hardcoded robot?
    if (rName == NULL || strcmp(rName, "kuka_lwr"))
    {
      continue;
    }
    break;
  }
  TiXmlHandle robotHandle = TiXmlHandle(robot);
  TiXmlElement* groupState = robotHandle.FirstChild("group_state").Element();
  for (;; groupState = groupState->NextSiblingElement())
  {
    if (!groupState)
    {
      break;
    }
    const char *gName = groupState->Attribute("name");
    const char *gGroup = groupState->Attribute("group");
    if (gName == NULL || strcmp(gName, poseName.c_str()))
    {
      continue;
    }
    robot->RemoveChild(groupState);
  }
  groupState = new TiXmlElement("group_state");
  robot->LinkEndChild(groupState);
  groupState->SetAttribute("name", poseName);
  // TODO hardcoded group?
  groupState->SetAttribute("group", "arm");
  writeJoints(groupState, joints);
  doc.SaveFile(filename);
}

void jointCallback(const sensor_msgs::JointState &state)
{
  map<string, double> joints;
  for (int i = 0; i < state.name.size(); i++)
  {
    string jointName = state.name[i];
    if (jointName.find("lwr") != jointName.npos)
    {
      joints[jointName] = state.position[i];
    }
  }
  printf("Captured joint state. (w)rite to file, (r)etry or (a)bort?\n");
  char answer = getchar();
  if (answer == 'a')
  {
    exit(EXIT_SUCCESS);
  }
  if (answer == 'w')
  {
    string srdf(kukaFile);
    string name;
    printf("Specify the name of the pose: ");
    cin >> name;
    writeSRDF(srdf, name, joints);
    exit(EXIT_SUCCESS);
  }
  if (answer != '\n')
  {
    while (getchar() != '\n')
      ;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_pose");
  if (argc < 2)
  {
    string ws(argv[0]);
    ws = ws.substr(0, ws.rfind("/lib"));
    ws = ws.substr(0, ws.rfind("/"));
    ws += "/src/euroc_manipulation/suturo_manipulation_moveit/config/kuka_lwr.srdf";
    printf("No srdf file specified, using %s\n", ws.c_str());
    kukaFile = ws.c_str();
  }
  else
  {
    kukaFile = argv[1];
  }
  printf("Listening for joint states...\n");
  ros::NodeHandle nh;
  ros::Subscriber hans_pansen = nh.subscribe("/joint_states", 1, jointCallback);
  ros::spin();
  return 0;
}
