#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import radians

def main():
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('write_nu', anonymous=True)

    group_name = "arm"  # Replace with your actual planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_max_velocity_scaling_factor(0.1)  
    move_group.set_max_acceleration_scaling_factor(0.1)

    waypoints = []

    start_pose = move_group.get_current_pose().pose

    # "N" waypoints
    n_start = geometry_msgs.msg.Pose()
    n_start.orientation = start_pose.orientation
    n_start.position.x = 0.3
    n_start.position.y = 0.0
    n_start.position.z = 0.2
    waypoints.append(n_start)

    n_up = geometry_msgs.msg.Pose()
    n_up.orientation = n_start.orientation
    n_up.position.x = 0.3
    n_up.position.y = 0.1
    n_up.position.z = 0.2
    waypoints.append(n_up)

    n_diag = geometry_msgs.msg.Pose()
    n_diag.orientation = n_up.orientation
    n_diag.position.x = 0.35
    n_diag.position.y = 0.0
    n_diag.position.z = 0.2
    waypoints.append(n_diag)

    n_end = geometry_msgs.msg.Pose()
    n_end.orientation = n_diag.orientation
    n_end.position.x = 0.35
    n_end.position.y = 0.1
    n_end.position.z = 0.2
    waypoints.append(n_end)

    # Transition between "N" and "U"
    transition = geometry_msgs.msg.Pose()
    transition.orientation = n_end.orientation
    transition.position.x = 0.35
    transition.position.y = 0.1
    transition.position.z = 0.3
    waypoints.append(transition)

    # "U" waypoints
    u_start = geometry_msgs.msg.Pose()
    u_start.orientation = transition.orientation
    u_start.position.x = 0.35
    u_start.position.y = 0.1
    u_start.position.z = 0.2
    waypoints.append(u_start)

    u_down = geometry_msgs.msg.Pose()
    u_down.orientation = u_start.orientation
    u_down.position.x = 0.35
    u_down.position.y = 0.0
    u_down.position.z = 0.2
    waypoints.append(u_down)

    u_bottom = geometry_msgs.msg.Pose()
    u_bottom.orientation = u_down.orientation
    u_bottom.position.x = 0.375
    u_bottom.position.y = 0.0
    u_bottom.position.z = 0.2
    waypoints.append(u_bottom)

    u_up = geometry_msgs.msg.Pose()
    u_up.orientation = u_bottom.orientation
    u_up.position.x = 0.375
    u_up.position.y = 0.1
    u_up.position.z = 0.2
    waypoints.append(u_up)

    # Compute the Cartesian path for "N" and "U"
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,   # Waypoints to follow
        0.01,        # eef_step
        0.0          # jump_threshold
    )

    rospy.loginfo("Planned trajectory fraction: %f" % fraction)

    if fraction > 0.9:
        move_group.execute(plan, wait=True)
        rospy.loginfo("Successfully executed 'N' and 'U' trajectory.")
    else:
        rospy.logerr("Failed to compute a valid trajectory for 'N' and 'U'.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
