#!/usr/bin/env python3
"""
Check if MoveIt IK service /compute_ik exists on the current ROS master.
Run in Gazebo container (or any terminal with same ROS_MASTER_URI):
  rosrun motion_control check_moveit_ik.py
Or:
  python3 scripts/check_moveit_ik.py   (from workspace, after source devel/setup.bash)
"""

import os
import sys
import rospy

def main():
    rospy.init_node("check_moveit_ik", anonymous=True)
    service_name = rospy.get_param("~service_name", "/compute_ik")
    timeout = rospy.get_param("~timeout", 2.0)
    print("ROS_MASTER_URI =", os.environ.get("ROS_MASTER_URI", "not set"))
    print("Checking for service:", service_name, "(timeout %.1fs)..." % timeout)
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        print("OK: %s exists (MoveIt IK is available)." % service_name)
        try:
            from rosservice import get_service_type_by_name
            st = get_service_type_by_name(service_name)
            print("     Type:", st)
        except Exception:
            pass
        return 0
    except rospy.ROSException as e:
        print("NOT FOUND: %s is not available." % service_name)
        print("  (Start MoveIt move_group in the Gazebo/MoveIt container if you need IK from MoveIt.)")
        return 1

if __name__ == "__main__":
    sys.exit(main())
