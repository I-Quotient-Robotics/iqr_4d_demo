# -*- coding: utf-8 -*-
#!/usr/bin/env python

import sys
import signal
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations

from std_msgs.msg import String
from mir_api import mir_move_goal, get_mir_goal
from moveit_bridge.msg import pose
from pan_tilt_driver.msg import PanTiltCmd
from srt_gripper_driver.msg import SRTGripperCmd

running = False

velocity_scaling = 0.15
acceleration_scaling = 0.4

def set_joints(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6):
    joint_goal = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    if group.go(joint_goal, wait=True):
        print 'ur5 move finished'
    
    return True

def set_tcp(pos_x, pos_y, pos_z, x, y, z, w):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x
    pose_target.position.y = pos_y
    pose_target.position.z = pos_z
    pose_target.orientation.x = x
    pose_target.orientation.y = y
    pose_target.orientation.z = z
    pose_target.orientation.w = w
    group.set_pose_target(pose_target)
    try:
        plan = group.plan()
    except:
        print 'ur5 plan error'
    rospy.sleep(2)
    if group.execute(plan, wait=True):
        print 'ur5 move finished'

    group.stop()

    return True

def signal_handler(signum, frame):
    global running
    running = False

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rospy.init_node("ur5_moveit_test")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5_arm")
    rospy.sleep(3)

    group.set_max_velocity_scaling_factor(velocity_scaling)
    group.set_max_acceleration_scaling_factor(acceleration_scaling)
    group.clear_pose_targets()

    # Home
    print 'move to home...'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)

    # PickA
    print 'move to pick A...'
    if set_tcp(0.6712, -0.0995, 1.2431, -0.0046, 0.7045, 0.0055, 0.7096):
        pass
    else:
        print 'move to pick A error'
        exit(-1)
    rospy.sleep(1)

    # PickB
    print 'move to pick B...'
    if set_tcp(0.6712, -0.0995, 0.9955, -0.0046, 0.7045, 0.0055, 0.7096):
        pass
    else:
        print 'move to pick B error'
        exit(-1)
    rospy.sleep(1)

    # Home
    print 'move to home...'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)

    print('All finished')

