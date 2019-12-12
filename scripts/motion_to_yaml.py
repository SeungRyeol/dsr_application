#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @brief    Save the settings for the motion function as a yaml file.
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
CWD = os.getcwd() + '/catkin_ws/src/dsr_application/config/'
JOG_VAL = 60

m_joyAnalogFlag = False
m_xyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = 0
m_joyJogVel = 0.0

def shutdown():
    print("shutdown time!")
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

r = CDsrRobot(ROBOT_ID, ROBOT_MODEL)

def joy_cb(msg):
    global m_joyAnalogFlag
    global m_joyButtonFlag
    global m_joyJogFlag
    global m_joyJogVel

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
            r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, 0)
            m_joyJogFlag = -1

if __name__ == "__main__":
    rospy.init_node('motion_to_yaml')
    rospy.on_shutdown(shutdown)

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)

    r.set_robot_mode(ROBOT_MODE_MANUAL) # init mode

    with open(CWD + 'joy_params.yaml') as f: # open joy config file
        conf = yaml.load(f)

    with open(CWD + 'motion_variable.yaml') as f: # Open an existing yaml file
        motion_dict = yaml.load(f)
        if motion_dict == None:
            motion_dict = {}

    while not rospy.is_shutdown():
        print("\n********** Motion to YAML **********")
        print("\n1. Test motion\n2. Save motion\n3. Quit\n")
        selection = raw_input("Your selection >> ")
        motion_type, motion_pos, motion_comment = {}, {}, {}
        waypoint = []

        if selection == '1': # Test motion
            key =  raw_input("Type the key-index to find >> ")

            if key in motion_dict:
                key_type =motion_dict[key]['type']
                if key_type == 'movej':
                    movej(motion_dict[key]['pos'], vel=50, acc=30)
                elif key_type == 'movel':
                    movel(motion_dict[key]['pos'], [50, 50], [100, 100])
                elif key_type == 'movesj':
                    movesj(motion_dict[key]['pos'], vel=50, acc=50)
                elif key_type == 'movesx':
                    movesx(motion_dict[key]['pos'], [100, 100], [100, 100])
                r.set_robot_mode(ROBOT_MODE_MANUAL)
            else:
                print("The key you are looking for does not exist.")
                exit(1)

        elif selection == '3': # End of program
            exit(1)

        else: # not test motion
            print("\n********** Select motion to save **********")
            print("1. movej\n2. movel\n3. movesj\n4. movesx\n5. quit\n")
            selection = raw_input("Your selection >> ")
            if selection == '1': # movej
                ans = raw_input("Type 's', if you want to save the current joint position >> ")
                if ans == 's':
                    motion_type['type'] = 'movej'
                    motion_pos['pos'] = list(r.get_current_pose(ROBOT_SPACE_JOINT).pos)
                    
            elif selection == '2': # movel
                ans = raw_input("Type 's', if you want to save the current position >> ")
                if ans == 's':
                    motion_type['type'] = 'movel'
                    motion_pos['pos'] = list(r.get_current_pose(ROBOT_SPACE_TASK).pos)

            elif selection == '3': # movesj
                while True:
                    print("\n'w': add waypoint\n's': finish typing waypoint")
                    ans = raw_input("Your command >> ")
                    if ans == 'w':
                        waypoint.append(posj(list(r.get_current_pose(ROBOT_SPACE_JOINT).pos)))
                        print("Waypoint added\n")
                    elif ans == 's':
                        motion_type['type'] = 'movesj'
                        motion_pos['pos'] = waypoint
                        break

            elif selection == '4': # movesx
                while True:
                    print("\n'w': add waypoint\n's': finish typing waypoint")
                    ans = raw_input("Your command >> ")
                    if ans == 'w':
                        print(type(r.get_current_pose(ROBOT_SPACE_TASK).pos))
                        waypoint.append(posx(list(r.get_current_pose(ROBOT_SPACE_TASK).pos)))
                        print("Waypoint added\n")
                    elif ans == 's':
                        motion_type['type'] = 'movesx'
                        motion_pos['pos'] = waypoint
                        break

            elif selection == '5': # quit:
                exit(1)

            # Save config
            key = raw_input("Type your key-index >> ")
            motion_comment['comment'] = raw_input("Please type a comment to add >> ")
            motion_dict[key] = motion_type
            motion_dict[key].update(motion_pos)
            motion_dict[key].update(motion_comment)

            with open(CWD + 'motion_variable.yaml', 'w') as datafile: # Saving motion plan
                yaml.dump(motion_dict, datafile, default_flow_style=False)

            rospy.loginfo("The task is saved as yaml file.")

    print 'good bye!'