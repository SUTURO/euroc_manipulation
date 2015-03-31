#ifndef MANIPULATION_NODE_STATUS
#define MANIPULATION_NODE_STATUS

#include <ros/ros.h>
#include <suturo_msgs/Task.h>
#include <suturo_startup_msgs/ManipulationNodeStatus.h>
#include <boost/format.hpp>
#include <typeinfo>

namespace suturo_manipulation
{
	class NodeStatus
	{
	public:
		NodeStatus(ros::NodeHandle &node_handle) : node_handle_(node_handle) {
			// logger = Logger("NodeStatus");
			node_status_publisher = node_handle_.advertise<suturo_startup_msgs::ManipulationNodeStatus> ("/suturo/manipulation_node_status", 1, true);
		}
		
		bool nodeStarted(int node)
		{
			// logger.logInfo((boost::format("trying to inform that node %s started") % node).str());
			suturo_startup_msgs::ManipulationNodeStatus status_msg;
			status_msg.started_node = node;
			node_status_publisher.publish(status_msg);
			return true;
		}
		
		bool publishRequiredNodes()
		{
			// node_status_publisher = node_handle_.advertise<suturo_perception_msgs::ManipulationNodeStatus> ("/suturo/manipulation_node_status", 1, true);
			suturo_startup_msgs::ManipulationNodeStatus status_msg;
			status_msg.started_node = suturo_startup_msgs::ManipulationNodeStatus::REQUIRED_NODES_INCOMING;
			status_msg.required_nodes.push_back(suturo_startup_msgs::ManipulationNodeStatus::NODE_JOINT_STATE);
			status_msg.required_nodes.push_back(suturo_startup_msgs::ManipulationNodeStatus::NODE_PUBLISH_OBJECT_FRAMES);
			node_status_publisher.publish(status_msg);
			return true;
		}
		
		// std::vector<unsigned short> getRequiredNodesForTask(int task)
		// {
		// 	std::vector<unsigned short> required_nodes;
			
		// 	switch (task)
		// 	{
		// 		case suturo_msgs::Task::TASK_4:
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_ODOM_COMBINER);
		// 		case suturo_msgs::Task::TASK_1:
		// 		case suturo_msgs::Task::TASK_2:
		// 		case suturo_msgs::Task::TASK_3:
		// 		case suturo_msgs::Task::TASK_5:
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_CLOUD_SCENE);
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_CLOUD_GRIPPER);
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_SCENE);
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_GRIPPER);
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_COLOR_RECOGNIZER);
		// 		break;
		// 		case suturo_msgs::Task::TASK_6:
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_CLOUD_GRIPPER);
		// 			required_nodes.push_back(suturo_perception_msgs::ManipulationNodeStatus::NODE_GRIPPER);
		// 		break;
		// 		default:
		// 			logger.logError((boost::format("Couldn't define which nodes are required! Unknown task_type: %s") % task).str());
		// 		break;
		// 	}
			
		// 	return required_nodes;
		// }
		
		
	protected:
		ros::NodeHandle node_handle_;
		ros::Publisher node_status_publisher;
		// Logger logger;
	};
}

#endif
