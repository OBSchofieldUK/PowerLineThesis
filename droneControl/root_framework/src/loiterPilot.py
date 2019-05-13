#!/usr/bin/env python

import rospy

import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (String, Int8)

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
targetWP = '/onboard/setpoint/loiter'


keySub = '/gcs/keypress'
class loiterPilot(): 
    def __init__(self):
        rospy.init_node('loiterPilot')
        self.rate = rospy.Rate(20)
        self.enable = False
        
        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'),mavSP.PoseStamped, self.onPositionChange)

        rospy.Subscriber(keySub, Int8, self._cb_onKeypress)
        
        self.targetPub = rospy.Publisher
        self.loiterPub = mavSP.get_pub_position_local(queue_size=5)

        self.loiterPos = mavSP.PoseStamped()
        self.curPos = mavSP.PoseStamped()

    def _pubMsg(self, msg, topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def _cb_onKeypress(self, msg):
        keypress = str(chr(msg.data))
        keypress.lower()
        if keypress == 'a':
            self.loiterPos.pose.position.x += 0.5 
        if keypress == 'd':
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
            #TODO rotation to euler adjustment 
            pass
        if keypress == 'e':
            #TODO rotation to euler adjustment 
            pass
    
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
