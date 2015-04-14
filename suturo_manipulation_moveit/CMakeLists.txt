cmake_minimum_required(VERSION 2.8.3)
project(suturo_manipulation_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib 
  actionlib_msgs
  tf
  moveit_ros_planning_interface
  euroc_c2_msgs
  sensor_msgs
  std_msgs
  suturo_manipulation_msgs
  suturo_startup_msgs
  suturo_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES perception_utils
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs suturo_manipulation_msgs suturo_msgs suturo_startup_msgs
  # DEPENDS opencv pcl
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# include_directories()
include_directories(
  SYSTEM 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}
  include
  # hack_for_groovy
#  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(state_publisher
#   src/${PROJECT_NAME}/state_publisher.cpp
# )
set(suturo_manipulation_muh
  include/suturo_manipulation_moveit/node_status.hpp
)
## Declare a cpp executable
# add_executable(state_publisher_node src/state_publisher_node.cpp)
add_executable(telemetry_to_joint_state src/telemetry-to-joint-state.cpp ${suturo_manipulation_muh})
add_executable(arm_controller src/arm_controller.cpp)
add_executable(arm_base_controller src/arm_base_controller.cpp)
add_executable(base_controller src/base_controller.cpp)
add_executable(arm_controller_servo src/arm_controller_servo.cpp)
add_executable(gripper_controller src/gripper_controller.cpp)
add_executable(spawn_planningscene src/spawn_planningscene.cpp)
add_executable(save_pose src/save_pose.cpp)
add_executable(publish_objects_tf_frames src/publish_objects_tf_frames.cpp ${suturo_manipulation_muh})

target_link_libraries(spawn_planningscene
  ${catkin_LIBRARIES}
  yaml-cpp
  )


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(state_publisher_node state_publisher_generate_messages_cpp)
add_dependencies(arm_controller ${catkin_EXPORTED_TARGETS} euroc_c2_msgs_generate_messages_cpp)
add_dependencies(base_controller ${catkin_EXPORTED_TARGETS} euroc_c2_msgs_generate_messages_cpp)
add_dependencies(arm_controller_servo ${catkin_EXPORTED_TARGETS} euroc_c2_msgs_generate_messages_cpp)
add_dependencies(arm_base_controller ${catkin_EXPORTED_TARGETS} euroc_c2_msgs_generate_messages_cpp)
add_dependencies(gripper_controller ${catkin_EXPORTED_TARGETS} euroc_c2_msgs_generate_messages_cpp)
add_dependencies(spawn_planningscene ${catkin_EXPORTED_TARGETS})
add_dependencies(save_pose ${catkin_EXPORTED_TARGETS})
add_dependencies(publish_objects_tf_frames ${catkin_EXPORTED_TARGETS} suturo_manipulation_msgs_generate_messages_cpp)
add_dependencies(telemetry_to_joint_state ${catkin_EXPORTED_TARGETS} suturo_manipulation_msgs)

## Specify libraries to link a library or executable target against
# target_link_libraries(state_publisher_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(telemetry_to_joint_state
  ${catkin_LIBRARIES}
)

target_link_libraries(arm_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(arm_base_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(base_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(arm_controller_servo
  ${catkin_LIBRARIES}
)

target_link_libraries(gripper_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(spawn_planningscene
  ${catkin_LIBRARIES}
)

target_link_libraries(save_pose
  ${catkin_LIBRARIES}
)

target_link_libraries(publish_objects_tf_frames
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
