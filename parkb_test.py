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
    
    if set_joints(-0.9689224402057093, -2.429319683705465, -1.675335709248678, 0.8453360795974731, 0.9908995628356934, 0.11598782241344452):
        pass
    else:
        print 'move to pick A error'
        exit(-1)
    rospy.sleep(2)
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
    if set_joints(-1.022879425679342, -2.551796261464254, -1.4628899733172815, 0.759224534034729, 1.044780969619751, 0.10859468579292297):
        pass
    else:
        print 'move to pick B error'
        exit(-1)
    rospy.sleep(1)
    
    set_gripper(1, 5)
    rospy.sleep(2)

    # PickC    
    print 'move to pick C...'
    # if set_tcp(0.9356, -0.0263, 1.0780, 0.0115, 0.7273, 0.0258, 0.6856):
    #     pass
    # else:
    #     print 'move to pick B error'
    #     exit(-1)
    # rospy.sleep(1)
    
    print 'move to pick C...'
    if set_joints(-1.075698200856344, -2.3347747961627405, -1.3744953314410608, 0.45704734325408936, 1.0972026586532593, 0.006229175720363855):
        pass
    else:
        print 'move to pick C error'
        exit(-1)
    rospy.sleep(1)
    
    # PickD    
    print 'move to pick D...'
    # if set_tcp(0.9356, -0.0263, 1.0780, 0.0115, 0.7273, 0.0258, 0.6856):
    #     pass
    # else:
    #     print 'move to pick B error'
    #     exit(-1)
    # rospy.sleep(1)
    
    print 'move to pick D...'
    if set_joints(-1.7616327444659632, -2.2878263632403772, -1.4145467917071741, 0.46018683910369873, 1.7794731855392456, -0.06513482729067022):
        pass
    else:
        print 'move to pick D error'
        exit(-1)
    rospy.sleep(1)
    
    # PickE    
    print 'move to pick E...'
    # if set_tcp(0.9356, -0.0263, 1.0780, 0.0115, 0.7273, 0.0258, 0.6856):
    #     pass
    # else:
    #     print 'move to pick B error'
    #     exit(-1)
    # rospy.sleep(1)
    
    print 'move to pick E...'
    if set_joints(-1.7806947867022913, -2.5459678808795374, -1.490572754536764, 0.794035792350769, 1.7985693216323853, -0.06716329256166631):
        pass
    else:
        print 'move to pick C error'
        exit(-1)
    rospy.sleep(1)
    
    set_gripper(2, 5)
    rospy.sleep(2)

    # Home
    print 'move to home...'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)
    set_gripper(0, 5)

    print('All finished')

