#!/usr/bin/env python

import rospy
from math import pi, radians, degrees
import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (String, Int8)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
targetWP = '/onboard/setpoint/loiter'

keySub = '/gcs/keypress'

debug = True
class loiterPilot(): 
    def __init__(self):
        rospy.init_node('loiterPilot')
        self.rate = rospy.Rate(20)
        self.enable = False
        
        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'),mavSP.PoseStamped, self.onPositionChange)

        rospy.Subscriber(keySub, Int8, self._cb_onKeypress)
        
        # self.targetPub = rospy.Publisher
        # self.loiterPub = mavSP.get_pub_position_local(queue_size=5)

        
        self.loiterPub = rospy.Publisher(targetWP, mavSP.PoseStamped, queue_size=5)
        self.loiterPos = mavSP.PoseStamped()
        self.curPos = mavSP.PoseStamped()

    def _pubMsg(self, msg, topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def adjustYaw(self, angle=5.0): 
        (_, _, yaw) = euler_from_quaternion([self.loiterPos.pose.orientation.x, self.loiterPos.pose.orientation.y, self.loiterPos.pose.orientation.z, self.loiterPos.pose.orientation.w])

        yaw += radians(angle)
        orientAdj = quaternion_from_euler(0, 0, yaw)

        self.loiterPos.pose.orientation.x = orientAdj[0]
        self.loiterPos.pose.orientation.y = orientAdj[1]
        self.loiterPos.pose.orientation.z = orientAdj[2]
        self.loiterPos.pose.orientation.w = orientAdj[3]


    def _cb_onKeypress(self, msg):
        keypress = str(chr(msg.data))
        keypress.lower()
        # print("I got:", keypress)
        if self.enable:
            if keypress == 'd':
                self.loiterPos.pose.position.x += 0.5 
            if keypress == 'a':
                self.loiterPos.pose.position.x -= 0.5
            if keypress == 'w':
                self.loiterPos.pose.position.y += 0.5
            if keypress == 's':
                self.loiterPos.pose.position.y -= 0.5
            if keypress == 'z':
                self.loiterPos.pose.position.z += 0.5 
            if keypress == 'x':
                self.loiterPos.pose.position.z -= 0.5
            if keypress == 'q':
                self.adjustYaw(5.0)
            if keypress == 'e':
                self.adjustYaw(-5.0)
            if debug: 
                if keypress == 'b':
                    self.loiterPos.pose.position.x = 0
                    self.loiterPos.pose.position.y = 20
                    self.loiterPos.pose.position.z = 7.5
                if keypress == 'h':
                    self.loiterPos.pose.position.x = 0
                    self.loiterPos.pose.position.y = 0
                    self.loiterPos.pose.position.z = 2

        else:
            options = "wasdqezxh"        #options as above
            if keypress in options:  
                print("warn: loiterpilot not enabled")
    
    def onStateChange(self, msg):
        if msg.data == 'loiter':
            print('loiter enabled')
            self.loiterPos = self.curPos
            self.enable = True
        else:
            if self.enable:
                print('loiter disabled')
            self.enable = False
        
    def onPositionChange(self,msg):
        self.curPos = msg

    def run(self):

        while not rospy.is_shutdown():
            if self.enable:
                self._pubMsg(self.loiterPos, self.loiterPub)
                self.rate.sleep()

if __name__ == "__main__":
    LP = loiterPilot()
    LP.run()
