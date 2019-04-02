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
from mir_api import mir_move_goal
from moveit_bridge.msg import pose
from srt_gripper_driver.msg import SRTGripperCmd

running = False

velocity_scaling = 0.15
acceleration_scaling = 0.4

PARK_A_GUID = '3c96dbb8-5219-11e9-82ef-94c691a737d7'
PARK_B_GUID = '6eadab0f-5219-11e9-82ef-94c691a737d7'
PARK_C_GUID = 'b44fd151-5219-11e9-82ef-94c691a737d7'
PARK_STOP_GUID = '3d1bfd29-5243-11e9-b6a0-94c691a737d7'

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

    rospy.init_node("picking_demo")
    gripper_pub = rospy.Publisher("/srt_gripper_driver_node/srt_gripper_cmd", SRTGripperCmd, queue_size=5)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("ur5_arm")
    rospy.sleep(3)

    group.set_max_velocity_scaling_factor(velocity_scaling)
    group.set_max_acceleration_scaling_factor(acceleration_scaling)
    group.clear_pose_targets()

    print('Step 1 Release gripper')
    set_gripper(0, 5)
    rospy.sleep(1)

    print 'Step 2 Send UR5 to home position'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)

    print 'Step 3 Send MiR to Park B position'
    if mir_move_goal(PARK_B_GUID):
        pass
    else:
        print 'go to Park B error'
        exit(-1)
    rospy.sleep(1)

    print 'Step 5 Send UR5 to pre pickup position'
    if set_joints(-0.9689224402057093, -2.429319683705465, -1.675335709248678, 0.8453360795974731, 0.9908995628356934, 0.11598782241344452):
        pass
    else:
        print 'move to pre pickup position error'
        exit(-1)
    rospy.sleep(2)
    
    print('Step 6 Open Gripper')
    set_gripper(2, 5)
    rospy.sleep(2)

    print 'Step 7 Send UR5 to pickup position'
    if set_joints(-1.022879425679342, -2.551796261464254, -1.4628899733172815, 0.759224534034729, 1.044780969619751, 0.10859468579292297):
        pass
    else:
        print 'move to pickup position error'
        exit(-1)
    rospy.sleep(1)
    
    print('Step 8 Close Gripper')
    set_gripper(1, 5)
    rospy.sleep(2)
    
    print('Setp 9 Send UR5 to after pickup position')
    if set_joints(-1.075698200856344, -2.3347747961627405, -1.3744953314410608, 0.45704734325408936, 1.0972026586532593, 0.006229175720363855):
        pass
    else:
        print 'move to after pickup position error'
        exit(-1)
    rospy.sleep(1)
    
    print('Setp 10 Send UR5 to pre dropoff position')
    if set_joints(-1.7616327444659632, -2.2878263632403772, -1.4145467917071741, 0.46018683910369873, 1.7794731855392456, -0.06513482729067022):
        pass
    else:
        print 'move to pre dropoff position error'
        exit(-1)
    rospy.sleep(1)
    
    print('Setp 11 Send UR5 to dropoff position')
    if set_joints(-1.7806947867022913, -2.5459678808795374, -1.490572754536764, 0.794035792350769, 1.7985693216323853, -0.06716329256166631):
        pass
    else:
        print 'move to dropoff position error'
        exit(-1)
    rospy.sleep(1)
    
    print('Step 12 Open Gripper')
    set_gripper(2, 5)
    rospy.sleep(2)

    print 'Step 13 Send UR5 to home position'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)
    
    print('Step 14 Release Gripper')
    set_gripper(0, 5)
    rospy.sleep(2)
    
    print 'Step 15 Send MiR to Park C position'
    if mir_move_goal(PARK_C_GUID):
        pass
    else:
        print 'go to Park C error'
        exit(-1)
    rospy.sleep(1)
    
    print 'Step 16 Send UR5 to pre pickup position'
    if set_joints(-1.4745848814593714, -2.058070484791891, -0.9536917845355433, -1.7607253233539026, 1.6295572519302368, 0.0742209255695343):
        pass
    else:
        print 'move to pre pickup position error'
        exit(-1)
    rospy.sleep(1)
    
    print('Step 17 Open Gripper')
    set_gripper(2, 5)
    rospy.sleep(2)
    
    print 'Step 18 Send UR5 to pickup position'
    if set_joints(-1.453597370778219, -2.0830958525287073, -1.4549830595599573, -1.2274516264544886, 1.6297969818115234, 0.09535660594701767):
        pass
    else:
        print 'move to pickup position error'
        exit(-1)
    rospy.sleep(1)
    
    print('Step 19 Close Gripper')
    set_gripper(1, 5)
    rospy.sleep(2)
    
    print 'Step 20 Send UR5 to home position'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)
    
    print 'Step 21 Send MiR to Park A position'
    if mir_move_goal(PARK_A_GUID):
        pass
    else:
        print 'go to Park A error'
        exit(-1)
    rospy.sleep(1)
    
    print 'Step 22 Send UR5 to pre dropoff position'
    if set_joints(-1.52523, -2.35885, -0.74139, -1.66245, 1.62397, 0.02839):
        pass
    else:
        print 'move to pre dropoff position error'
        exit(-1)
    rospy.sleep(1)
    
    print 'Step 23 Send UR5 to dropoff position'
    if set_joints(-1.50926, -2.46715, -1.15370, -1.14110, 1.62479, 0.044251):
        pass
    else:
        print 'move to dropoff position error'
        exit(-1)
    rospy.sleep(1)
    
    print('Step 24 Open Gripper')
    set_gripper(2, 5)
    rospy.sleep(2)
    
    print 'Step 25 Send UR5 to home position'
    if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
        pass
    else:
        print 'move to home error'
        exit(-1)
    rospy.sleep(1)
    
    print('Step 26 Release gripper')
    set_gripper(0, 5)
    rospy.sleep(1)
    
    print 'Step 27 Send MiR to Park Stop position'
    if mir_move_goal(PARK_STOP_GUID):
        pass
    else:
        print 'go to Park Stop error'
        exit(-1)
    rospy.sleep(1)
    
    print('All finished')


