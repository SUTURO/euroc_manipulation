cmake_minimum_required(VERSION 2.8.3)
project(suturo_manipulation_moveit)

find_package(catkin REQUIRED COMPONENTS
  euroc_c2_msgs
  roscpp
  rospy
  actionlib 
  actionlib_msgs
  tf
  moveit_ros_planning_interface
)

find_package(Eigen REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

## Declare a cpp library
# add_library(state_publisher
#   src/${PROJECT_NAME}/state_publisher.cpp
# )

## Declare a cpp executable
# add_executable(state_publisher_node src/state_publisher_node.cpp)
add_executable(telemetry_to_joint_state src/telemetry-to-joint-state.cpp)
add_executable(arm_controller src/arm_controller.cpp)
add_executable(gripper_controller src/gripper_controller.cpp)
add_executable(spawn_planningscene src/spawn_planningscene.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(state_publisher_node state_publisher_generate_messages_cpp)
add_dependencies(arm_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(gripper_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(spawn_planningscene ${catkin_EXPORTED_TARGETS})

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

target_link_libraries(gripper_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(spawn_planningscene
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
