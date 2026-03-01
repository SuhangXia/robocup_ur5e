cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  moveit_msgs
  trajectory_msgs
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    moveit_msgs
    trajectory_msgs
    common_msgs
)

catkin_install_python(PROGRAMS
  nodes/aco_rrtstar_planner_node.py
  nodes/test_aco_rrtstar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
