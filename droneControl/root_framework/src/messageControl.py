#!/usr/bin/env python

import math
import utm
import rospy

import mavros.utils
import mavros.setpoint as mavSP
import mavros.command as mavCMD

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (Bool, String, Int8)
from sensor_msgs.msg import NavSatFix
from inspec_msg.msg import line_control_info

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'

loiterSub = '/onboard/setpoint/loiter'
missionSub = '/onboard/setpoint/mission'
inspectSub = '/onboard/setpoint/inspect'

homeSub = '/onboard/position/home'
homeReqPub = '/onboard/request/gpsHome'
wpCompleteTopic = '/onboard/check/WPSuccess'


class msgControl():
    def __init__(self):
        rospy.init_node('msgControl')
        self.rate = rospy.Rate(20)
        self.enable = False
        
        self.sysState = None
        self.homePos = None
        self.gotMission = False

        self.loiterMsg = mavSP.PoseStamped()
        self.pylonNavMsg = None

        self.setpoint = mavSP.PoseStamped()
        self.curLocalPos = mavSP.PoseStamped()
        
        self.setpointPub = mavSP.get_pub_position_local(queue_size=1)
        self.wpCompletePub = rospy.Publisher(wpCompleteTopic, Int8, queue_size=1)
        self.homeGPSPub = rospy.Publisher(homeReqPub, Bool, queue_size=1) 

        # root_framework Subs
        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber('/onboard/messageEnable',Bool, self.handlerEnable)
        
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self._cb_localPosUpdate)
        rospy.Subscriber(mavros.get_topic('home_position','home'), mavros_msgs.msg.HomePosition, self._cb_onHomeUpdate)


        # pilot subs
        rospy.Subscriber(loiterSub, mavSP.PoseStamped, self.pilot_loiterMsg)
        rospy.Subscriber(missionSub, NavSatFix, self.pilot_pylonNavMsg)
        rospy.Subscriber(inspectSub,line_control_info, self._onInspectPosUpdate)

    def handlerEnable(self,msg):
        if msg.data == True:
            print("messageHandler Enabled")
            self.enable = True
        if msg.data == False:
            if self.curLocalPos.pose.position.z > 0.1:
                print("message Handler: unable to disable - airbourne")
                self.sysState = 'loiter'
            else:
                print("messageHandler Disabled")
                self.enable = False

    def _pubMsg(self, msg, topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def onStateChange(self, msg):
        if msg.data == 'mission':
            self.pylonNavMsg = None
            self.gotMission = False
            self.sysState = 'missionHold'
        if msg.data == 'idle':
            self.enable = False
            print("messageHandler Disabled")
        else:
            self.sysState = msg.data

    def _cb_localPosUpdate(self, msg):
        self.curLocalPos = msg

    def _cb_onHomeUpdate(self, msg):
        if self.homePos == None:
            print("home updated")
        self.homePos = msg

    def onPositionChange(self, msg):
        self.curLocalPos = msg
    
# pilotMessage Handler
    def pilot_loiterMsg(self, msg):
        self.loiterMsg = msg

    def pilot_pylonNavMsg(self, msg):
        x,y,z = self.gpsToLocal(msg)
        tmpSP = mavSP.PoseStamped()
        tmpSP.pose.position.x = y
        tmpSP.pose.position.y = x
        tmpSP.pose.position.z = z
        tmpSP.pose.orientation =  self.curLocalPos.pose.orientation

        self.pylonNavMsg = tmpSP

        self.sysState = 'mission'
        self.gotMission = True

    def _onInspectPosUpdate(self,msg):
        
        pass

    def get_pilotMsg(self):
        outwardMsg = mavSP.PoseStamped()
        outwardMsg = self.loiterMsg

        if self.sysState == 'loiter':
            outwardMsg = self.loiterMsg

        if self.sysState == 'missionHold':
            outwardMsg = self.loiterMsg

        if self.sysState == 'mission':
            if (self.pylonNavMsg == None) or (self.gotMission == False):
                outwardMsg = self.loiterMsg
            else:
                outwardMsg = self.pylonNavMsg

        if self.sysState == 'inspect':
            pass

        self.setpoint = outwardMsg

# Waypoint checking
    def waypointCheck(self):
        
        if self.homePos == None:
            print("unable: no home position")
        else:
            self.get_pilotMsg()

            altCheck, setPointPos = self.altitudeCheck()
            proxCheck = self.proximityCheck()
            self._pubMsg(setPointPos, self.setpointPub)

            wpStatus = 0
            if altCheck:
                wpStatus = 1 # correct Altitude
                if proxCheck:
                    wpStatus = 2
            
            acceptedStates = "mission, inspect"
            if (self.sysState in acceptedStates):
                if self.sysState == 'mission' and self.gotMission:
                    self.wpCompletePub.publish(wpStatus)


    def altitudeCheck(self):
        preMsg = mavSP.PoseStamped()
        altCheck = False

        # print "curPose:", self.curLocalPos.pose
        # print "setPose:", self.setpoint.pose
        if abs(self.curLocalPos.pose.position.z - self.setpoint.pose.position.z) > 0.5:
            preMsg.pose.position.x = self.curLocalPos.pose.position.x
            preMsg.pose.position.y = self.curLocalPos.pose.position.y
            preMsg.pose.position.z = self.setpoint.pose.position.z
            altCheck = False
        else:
            preMsg.pose.position.x = self.setpoint.pose.position.x
            preMsg.pose.position.y = self.setpoint.pose.position.y
            preMsg.pose.position.z = self.setpoint.pose.position.z
            altCheck = True
        
        preMsg.pose.orientation = self.setpoint.pose.orientation
        return altCheck, preMsg

    def proximityCheck(self):
        if (abs(self.curLocalPos.pose.position.x - self.setpoint.pose.position.x) < 1.0) and (abs(self.curLocalPos.pose.position.y - self.setpoint.pose.position.y) < 1.0) :
            return True             
        else:
            return False

# Positioning calcuations
    def calcDist(self, utmPosA, utmPosB):
        dist = -1
        deltaEast = utmPosA[0]-utmPosB[0]
        deltaNorth = utmPosA[1]-utmPosB[1]
        if deltaNorth != 0:
            dist = math.sqrt(deltaNorth**2 + deltaEast**2)
        return deltaNorth, deltaEast, dist

    def gpsToLocal(self, gpsPos):
        utmPos = utm.from_latlon(gpsPos.latitude, gpsPos.longitude)

        utmHome = utm.from_latlon(self.homePos.geo.latitude, self.homePos.geo.longitude)

        deltaNorth, deltaEast, _ = self.calcDist(utmPos, utmHome)
        deltaAlt = gpsPos.altitude
        # print ("Local Point: %.4f, %.4f, %.2f" % (deltaNorth, deltaEast, gpsPos.altitude))
        return deltaNorth, deltaEast, deltaAlt

    def run(self):

        while not rospy.is_shutdown():
            
            if self.enable:
                self.waypointCheck()
            self.rate.sleep()


if __name__ == "__main__":
    LP = msgControl()
    LP.run()
