cmake_minimum_required(VERSION 2.8.3)
project(suturo_manipulation_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  suturo_msgs
  suturo_perception_msgs
  moveit_ros_planning_interface
)

find_package(Eigen REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp suturo_perception_msgs suturo_msgs)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

add_executable(perception_to_planningscene src/perception_to_planningscene.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(state_publisher_node state_publisher_generate_messages_cpp)
add_dependencies(perception_to_planningscene ${catkin_EXPORTED_TARGETS} suturo_perception_msgs_generate_messages_cpp suturo_msgs_generate_messages_cpp)

target_link_libraries(perception_to_planningscene ${catkin_LIBRARIES})