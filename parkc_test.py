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

def set_gripper(cmd, gear):
    gripper_cmd = SRTGripperCmd()
    gripper_cmd.openFlage = False
    gripper_cmd.closeFlage = False
    gripper_cmd.releaseFlage = False
    gripper_cmd.openGears = gear
    gripper_cmd.closeGears = gear

    if cmd==0:
        gripper_cmd.releaseFlage = True
    elif cmd==1:
        gripper_cmd.closeFlage = True
    elif cmd==2:
        gripper_cmd.openFlage = True

    gripper_pub.publish(gripper_cmd)

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
    gripper_pub = rospy.Publisher("/srt_gripper_driver_node/srt_gripper_cmd", SRTGripperCmd, queue_size=5)
    rospy.sleep(2)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5_arm")
    rospy.sleep(3)

    group.set_max_velocity_scaling_factor(velocity_scaling)
    group.set_max_acceleration_scaling_factor(acceleration_scaling)
    group.clear_pose_targets()

    # Home
    set_gripper(0, 5)
    rospy.sleep(1)

    print 'move to home...'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)

    # PickA
    print 'move to pick A...'
    # if set_tcp(0.9520, -0.0387, 1.2848, 0.0117, 0.7293, 0.0260, 0.6835):
    #     pass
    # else:
    #     print 'move to pick A error'
    #     exit(-1)
    # rospy.sleep(1)
    
    if set_joints(-1.4745848814593714, -2.058070484791891, -0.9536917845355433, -1.7607253233539026, 1.6295572519302368, 0.0742209255695343):
        pass
    else:
        print 'move to pick A error'
        exit(-1)
    rospy.sleep(1)

    set_gripper(2, 5)
    rospy.sleep(2)

    # PickB    
    print 'move to pick B...'
    # if set_tcp(0.9356, -0.0263, 1.0780, 0.0115, 0.7273, 0.0258, 0.6856):
    #     pass
    # else:
    #     print 'move to pick B error'
    #     exit(-1)
    # rospy.sleep(1)
    
    print 'move to pick B...'
    if set_joints(-1.453597370778219, -2.0830958525287073, -1.4549830595599573, -1.2274516264544886, 1.6297969818115234, 0.09535660594701767):
        pass
    else:
        print 'move to pick B error'
        exit(-1)
    rospy.sleep(1)
    
    set_gripper(1, 5)
    rospy.sleep(2)

    # Home
    print 'move to home...'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(5)
    set_gripper(0, 5)

    print('All finished')

