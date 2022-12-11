#!/usr/bin/python3

import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander
from std_msgs.msg import String
import rospy
from moveit_commander.conversions import pose_to_list

import sys

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_test', anonymous=False)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "sting_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.0
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.0
    move_group.set_start_state_to_current_state()
    move_group.set_pose_reference_frame("tf")
    move_group.set_pose_target(pose_goal)
    _, plan,_,_ = move_group.plan()

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    move_group.execute(plan)
    move_group.clear_pose_targets()

    exit()

    # input()


