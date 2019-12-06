#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @brief    The dsr_application Python package
# @author   Seung Ryeol Shin (dev_shin@cwsfa.co.kr)   

import rospy
import os
import sys
import yaml
from sensor_msgs.msg import Joy
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../doosan-robot/common/imp")) ) # get import pass : DSR_ROBOT.py 

# for single robot
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *
CWD = os.getcwd() + '/catkin_ws/src/dsr_application/scripts/'
JOG_VAL = 60

m_joyAnalogFlag = False
m_xyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = 0
m_joyJogVel = 0.0

# Saved Pos
HOME_POS = posj()
target_pos = [0, 0, 90, 0, 90, 0]
target_dict, index = {}, 0

def shutdown():
    print("shutdown time!")
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

r = CDsrRobot(ROBOT_ID, ROBOT_MODEL)

def joy_cb(msg):
    global index # pos dict index
    global target_pos
    global m_joyAnalogFlag
    global m_xyCompareFlag
    global m_joyButtonFlag
    global m_joyJogFlag
    global m_joyJogVel

    # Save Pos
    if msg.buttons[conf['L1_BUTTON']] == 1 and msg.buttons[conf['R1_BUTTON']] == 1:
        target_pos = list(r.get_current_pose(ROBOT_SPACE_JOINT).pos)
        target_dict[index] = target_pos
        index += 1
        target_dict['index'] = index
        with open(CWD + 'target_data.yaml', 'w') as outfile:
            yaml.dump(target_dict, outfile, default_flow_style=False) 
        rospy.loginfo("POSE IS SAVED")

    # Go to Target
    if msg.buttons[conf['A_BUTTON']] == 1 and msg.buttons[conf['B_BUTTON']] == 1:
        r.set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        r.movej(target_pos, 50, 50)
        r.set_robot_mode(ROBOT_MODE_MANUAL)

    # Go to HOME
    elif msg.buttons[conf['HOME_BUTTON']] == 1:
        r.set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        r.movej(HOME_POS, 50, 50)
        r.set_robot_mode(ROBOT_MODE_MANUAL)

    # joyAnalogFlag setting (JOINT2 ... JOINT5)
    if msg.axes[conf['JOINT2']] or msg.axes[conf['JOINT3']] or msg.axes[conf['JOINT4']] or msg.axes[conf['JOINT5']]:
        m_joyAnalogFlag = True
    else:
        m_joyAnalogFlag = False

    # joyButtonFlag setting (JOINT1, JOINT6)
    if msg.axes[conf['JOINT1_LEFT']] != 1.0 or msg.axes[conf['JOINT1_RIGHT']] != 1.0 \
        or msg.buttons[conf['JOINT6_C']] != 0 or msg.buttons[conf['JOINT6_CC']] != 0:
        m_joyButtonFlag = True
    else:
        m_joyButtonFlag = False

    if m_joyAnalogFlag or m_joyButtonFlag:

        if msg.axes[conf['JOINT1_LEFT']] < 1 or msg.axes[conf['JOINT1_RIGHT']] < 1: # For JOINT1
            m_joyJogFlag = 0
            m_joyJogVel  = JOG_VAL * (1 if msg.axes[conf['JOINT1_LEFT']] < 1 else -1 if msg.axes[conf['JOINT1_RIGHT']] < 1 else 0)
            
        if msg.axes[conf['JOINT2']] != 0: # For JOINT2
            m_joyJogFlag = 1
            m_joyJogVel  = JOG_VAL * (1 if msg.axes[conf['JOINT2']]<0 else -1)
                
        if msg.axes[conf['JOINT3']] != 0: # For JOINT3
            m_joyJogFlag = 2
            m_joyJogVel  = JOG_VAL * (1 if msg.axes[conf['JOINT3']]<0 else -1)
                
        if msg.axes[conf['JOINT4']] != 0: # For JOINT4
            m_joyJogFlag = 3
            m_joyJogVel  = JOG_VAL * (1 if msg.axes[conf['JOINT4']]<0 else -1)

        if msg.axes[conf['JOINT5']] != 0: # For JOINT5
            m_joyJogFlag = 4
            m_joyJogVel  = JOG_VAL * (1 if msg.axes[conf['JOINT5']]<0 else -1)

        if msg.buttons[conf['JOINT6_C']] != 0 or msg.buttons[conf['JOINT6_CC']] != 0: # For JOINT6
            m_joyJogFlag = 5
            m_joyJogVel  = JOG_VAL * (1 if msg.buttons[conf['JOINT6_C']] != 0 else -1 if msg.buttons[conf['JOINT6_CC']] != 0 else 0)

        r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, m_joyJogVel)

    else:
        if not m_joyAnalogFlag and not m_joyButtonFlag:
            rospy.loginfo("jog stop")
            r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, 0)
            m_joyJogFlag = -1

if __name__ == "__main__":
    rospy.init_node('teleop_joy_dsr')
    rospy.on_shutdown(shutdown)

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)

    with open(CWD + 'target_data.yaml', 'w') as datafile: # init target data file
        yaml.dump(target_dict, datafile, default_flow_style=False)

    with open(CWD + 'joy_params.yaml') as f: # open joy config file
        conf = yaml.load(f)

    r.set_robot_mode(ROBOT_MODE_MANUAL) # init mode

    while not rospy.is_shutdown(): pass

    print 'good bye!'