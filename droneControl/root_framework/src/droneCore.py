#!/usr/bin/env python

import rospy

import mavros as mav
import mavros.utils
import mavros.command as mavCMD
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import (String, Int8, Bool)

mavros.set_namespace('mavros')
# mavros commands for 


onB_StateSub = '/onboard/state'
keySub = '/gcs/keypress'

linefollowsub = '/onboard/setpoint/linefollow'
homePubTopic = '/onboard/position/home'
wpDoneSub = '/onboard/check/WPSuccess'
homeReqPub = '/onboard/request/gpsHome'

class droneCore():
    def __init__(self):
        rospy.init_node('droneCMD_node')
        self.rate = rospy.Rate(20)
        
        self.homeCoord = None
        self.gpsPos = NavSatFix()
        self.curPos = mavSP.PoseStamped()
        self.loiterPos = mavSP.PoseStamped()

        self.uavState = mavros_msgs.msg.State()
        self.setPoint = mavSP.PoseStamped()

        self.droneArmed = False
        self.loiter = False
        self.isAirbourne = False
        self.sysState = None
            
        # Publishers/Subscribers

        rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self._cb_uavState)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self._cb_localPos)
        rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, self._cb_SatFix)
        rospy.Subscriber(keySub,Int8, self._cb_onKeypress)
        rospy.Subscriber(wpDoneSub, Bool, self._stateComplete)
        rospy.Subscriber(homeReqPub, Bool, self.onHomeRequest)

        self.statePub = rospy.Publisher(onB_StateSub, String, queue_size=1)
        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)
        self.homePub = rospy.Publisher(homePubTopic, NavSatFix, queue_size=1)

        # Services 
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        



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
        self.curPos = msg
        if self.uavState.armed and (self.curPos.pose.position.z > 0.25):
            self.isAirbourne = True
        # self.curPos = self._genPoseMsg(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)        
        pass
    def _cb_SatFix(self,msg):
        self.gpsPos = msg

    def onHomeRequest(self, msg):
        if msg.data == True:
            self.homePub.publish(self.homeCoord)
    
    def _cb_onKeypress(self, msg):
        keypress = str(chr(msg.data))
        keypress.lower()
        if keypress == 't':
            self.droneTakeoff()
            self.droneLoiter()
        if keypress == 'l':
            if not self.isAirbourne:
                print ("Warn: Not airbourne!")
            else:
                self.droneLoiter()

        if keypress == 'm':
            if not self.isAirbourne:
                self.droneTakeoff(3.0)
            self.missionEnable()
        
        if keypress == 'h':
            self.homeCoord = self.gpsPos
            self.homePub.publish(self.homeCoord)

    def _stateComplete(self,msg):
        if (msg.data == True):
            if self.sysState == 'mission': 
                self.sysState = 'loiter'
                self.statePub.publish(self.sysState)

    def droneTakeoff(self, alt=1.0):
        if not self.isAirbourne:

            if not self.droneArmed:
                mavCMD.arming(True)
            
            preArmMsgs = self.curPos
            preArmMsgs.pose.position.z = alt
            # preArmMsgs = self._genPoseMsg(0,0,alt)
            # print(preArmMsgs)
            for i in range(50):
                self._pubMsg(preArmMsgs, self.spLocalPub)

            # if self.homeCoord == None:
            #     self.homeCoord = self.gpsPos
            #     self.homePub.publish(self.homeCoord)
            #     print("home updated")

            self.setMode(0,'OFFBOARD')
            self.setPoint = preArmMsgs

            self.isAirbourne = True

            #wait until takeoff has occurred
            while(self.curPos.pose.position.z <= (preArmMsgs.pose.position.z-0.25)):
                self._pubMsg(self.setPoint, self.spLocalPub)
        else:
            print('landing')

            
    def droneLoiter(self):
        # print('loiter enable')
        self.statePub.publish('loiter')
        # self.loiterPos = self.curPos
        self.loiter = True
        
    def missionEnable(self):
        self.statePub.publish('mission')
        self.sysState = 'mission'
        pass
    
    def run(self):
        if self.homeCoord == None:
            self.homeCoord = self.gpsPos
            print("Home Updated.")
        while not rospy.is_shutdown():
            self.rate.sleep()
        pass


if __name__ == "__main__":
    uavCore = droneCore()
    uavCore.run()
    pass
