#!/usr/bin/env python

import rospy
import mavros
import mavros.utils
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv

import sys

from std_msgs.msg import (Int8, String)

keySub = '/gcs/keypress'
statePub = '/gcs/state'
uavState = mavros_msgs.msg.State()

gcsInternalState = String()

mavros.set_namespace('mavros')

set_armDisarm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
set_uasMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

statePublisher = rospy.Publisher(statePub, String)

def _state_callback(topic):
    uavState.armed = topic.armed
    uavState.connected = topic.connected
    uavState.mode = topic.mode
    uavState.guided = topic.guided

def setArmTakeoff(state=False):
    mavros.command.arming(True)

    set_uasMode(0, 'AUTO.TAKEOFF')

    pass

def setLoiter():
    set_uasMode(0, 'AUTO.LOITER')
    global gcsInternalState
    if gcsInternalState != 'loiter':
        gcsInternalState='loiter'


def onKeyPress(msg):
    keyPress = str(chr(msg.data))
    keyPress.lower()
    if keyPress == "t": # Takeoff
        setArmTakeoff()
        setLoiter()
    if keyPress == "m":
        print('Mission')
    pass

def main():
    global gcsInternalState
    rospy.init_node('drome_cmd', anonymous=True)
    rate = rospy.Rate(20)

    subState = rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, _state_callback)
    keyGovern = rospy.Subscriber(keySub,Int8, onKeyPress)

    print ("connecting to FCU...")

    for i in range(12):
        if i >= 10 and not uavState.connected:
            print("connection timeout!")
            rospy.signal_shutdown('User quit')
            sys.exit()
            break
        elif uavState.connected:
            break
        else:
            rospy.sleep(1.)
    print("connected")

    gcsInternalState = 'idle'
    
    while not (rospy.is_shutdown()):
        # print("State: connected= %d, \t mode: %s" % (uavState.connected, uavState.mode) )
        statePublisher.publish(gcsInternalState)
        rate.sleep()

if __name__ == '__main__':
    main()
