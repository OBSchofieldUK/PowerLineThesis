#!/usr/bin/env python

import rospy

import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import String

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'

class loiterPilot():
    def __init__(self):
        rospy.init_node('loiterPilot')
        self.rate = rospy.Rate(20)
        self.enable = False
        
        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'),mavSP.PoseStamped, self.onPositionChange)
        
        self.loiterPub = mavSP.get_pub_position_local(queue_size=5)

        self.loiterPos = mavSP.PoseStamped()
        self.curPos = mavSP.PoseStamped()

    def _pubMsg(self, msg, topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()
    
    def onStateChange(self, msg):
        if msg.data == 'loiter':
            print('loiter enabled')
            self.loiterPos = self.curPos
            self.enable = True
        else:
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