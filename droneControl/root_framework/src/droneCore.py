#!/usr/bin/env python

import rospy

import mavros as mav
import mavros.utils
import mavros.command as mavCMD
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String

mavros.set_namespace('mavros')
# mavros commands for 

class droneCore():
    def __init__(self):
        rospy.init_node('droneCMD_node')
        self.rate = rospy.Rate(20)
        self.loiter = False
        # Publishers/Subscribers

        rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self._cb_uavState)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self._cb_localPos)

        self.statePub = rospy.Publisher('/onboard/state', String, queue_size=1)
        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)
        # Services 
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        
        self.curPos = mavSP.PoseStamped()
        self.loiterPos = mavSP.PoseStamped()

        self.uavState = mavros_msgs.msg.State()
        self.setPoint = mavSP.PoseStamped()

        self.droneArmed = False

    # ROS-Specific functions 
    def _genPoseMsg(self, x, y, z):
        poseMsg = mavros.setpoint.PoseStamped()
        poseMsg.pose.position.x = x
        poseMsg.pose.position.y = y
        poseMsg.pose.position.z = z
        poseMsg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())
        
        return poseMsg
    
    def _pubMsg(self,msg,topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()
        # print (msg)

    def _cb_uavState(self, msg):
        self.uavState = msg
        # print self.uavState
        pass

    def _cb_localPos(self, msg):
        # self.curPos = msg
        self.curPos = self._genPoseMsg(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)        
        pass


    def droneTakeoff(self, alt=3.0):
        if not self.droneArmed:
            mavCMD.arming(True)

        preArmMsgs = self._genPoseMsg(0,0,alt)
        for i in range(100):
            self._pubMsg(preArmMsgs, self.spLocalPub)

        self.setMode(0,'OFFBOARD')
        self.setPoint = preArmMsgs

        #wait until takeoff has occurred
        while(self.curPos.pose.position.z <= (preArmMsgs.pose.position.z-0.25)):
            self._pubMsg(self.setPoint, self.spLocalPub)


    def droneLoiter(self):
        print('loiter enable')
        self.statePub.publish('loiter')
        self.loiterPos = self.curPos
        self.loiter = True
        
    
    def run(self):
        self.droneTakeoff()
        self.droneLoiter()

        while not rospy.is_shutdown():

            if self.loiter:
                self._pubMsg(self.loiterPos, self.spLocalPub)
            else:
                self._pubMsg(self.setPoint, self.spLocalPub)
            self.rate.sleep()
        pass


if __name__ == "__main__":
    uavCore = droneCore()
    uavCore.run()
    pass
