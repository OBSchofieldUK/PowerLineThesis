#!/usr/bin/env python

import utm
import math
import rospy

import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP
import mavros.command as mavCMD

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (Bool, String)
from sensor_msgs.msg import NavSatFix
from inspec_msg.msg import line_control_info

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'

loiterSub = '/onboard/setpoint/loiter'
missionSub = '/onboard/setpoint/mission'
inspectSub = '/onboard/setpoint/inspect'

homeSub = '/onboard/position/home'
homeReqPub = '/onboard/request/gpsHome'

class msgControl():
    def __init__(self):
        rospy.init_node('msgControl')
        self.rate = rospy.Rate(20)

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        # rospy.Subscriber(loiterSub, mavSP.PoseStamped, self.cb_loiterMsg)
        rospy.Subscriber(missionSub, NavSatFix, self._cb_missionUpdate)
        rospy.Subscriber(homeSub, NavSatFix, self._cb_onHomeUpdate)
        rospy.Subscriber(inspectSub,line_control_info, self._onInspectPosUpdate)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self._cb_localPosUpdate)
        # self.targetPub = rospy.Publisher

        self.setpointPub = mavSP.get_pub_position_local(queue_size=5)
        self.wpCompletePub = rospy.Publisher('/onboard/check/WPSuccess', Bool, queue_size=1)
        self.homeGPSPub = rospy.Publisher(homeReqPub, Bool, queue_size=1) 

        self.homePos = None
        self.enable = False
        self.setpoint = mavSP.PoseStamped()
        self.curLocalPos = mavSP.PoseStamped()

    def _pubMsg(self, msg, topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def onStateChange(self, msg):
        if msg.data == 'loiter':

            pass

        if msg.data == 'mission':
            self.enable = True
            pass
    
    def _cb_localPosUpdate(self, msg):
        self.curLocalPos = msg

    def _cb_onHomeUpdate(self, msg):
        self.homePos = msg

    def onPositionChange(self, msg):
        self.curPos = msg
    
    def _cb_missionUpdate(self, msg):

        x,y,z = self.gpsToLocal(msg)
        self.setpoint.pose.position.x = y
        self.setpoint.pose.position.y = x
        self.setpoint.pose.position.z = z

    def _onInspectPosUpdate(self,msg):
        # msg.dist_XY
        # msg.dist_Z
        # msg.yaw
        pass

    def waypointCheck(self):
        altCheck, setPointPos = self.altitudeCheck()
        proxCheck = self.proximityCheck()
        print ("Alt: %r \t Prox: %r" % (altCheck, proxCheck))
        self._pubMsg(setPointPos, self.setpointPub)
        if proxCheck:
            self.wpCompletePub.publish(True)

    def altitudeCheck(self):
        preMsg = mavSP.PoseStamped()
        altCheck = False
        if abs(self.curLocalPos.pose.position.z - self.setpoint.pose.position.z) > 0.5:
            preMsg.pose.position.x = self.curLocalPos.pose.position.x
            preMsg.pose.position.y = self.curLocalPos.pose.position.y
            preMsg.pose.position.z = self.setpoint.pose.position.z
            preMsg.pose.orientation = self.curLocalPos.pose.orientation
            altCheck = False
        else:
            preMsg.pose.position.x = self.setpoint.pose.position.x
            preMsg.pose.position.y = self.setpoint.pose.position.y
            preMsg.pose.position.z = self.setpoint.pose.position.z
            altCheck = True
        
        return altCheck, preMsg

    def proximityCheck(self):
        if (abs(self.curLocalPos.pose.position.x - self.setpoint.pose.position.x) < 1.0) and (abs(self.curLocalPos.pose.position.y - self.setpoint.pose.position.y) < 1.0) :
            return True             
        else:
            return False

    def calcDist(self, utmPosA, utmPosB):
        dist = -1
        deltaEast = utmPosA[0]-utmPosB[0]
        deltaNorth = utmPosA[1]-utmPosB[1]
        if deltaNorth != 0:
            dist = math.sqrt(deltaNorth**2 + deltaEast**2)
        return deltaNorth, deltaEast, dist

    def gpsToLocal(self, gpsPos):
        utmPos = utm.from_latlon(gpsPos.latitude, gpsPos.longitude)

        utmHome = utm.from_latlon(self.homePos.latitude, self.homePos.longitude)

        deltaNorth, deltaEast, _ = self.calcDist(utmPos, utmHome)
        deltaAlt = gpsPos.altitude
        # print ("Local Point: %.4f, %.4f, %.2f" % (deltaNorth, deltaEast, gpsPos.altitude))
        return deltaNorth, deltaEast, deltaAlt

    def run(self):
        if self.homePos == None:
            self.homeGPSPub.publish(True)
        while not rospy.is_shutdown():
            if self.enable:
                self.waypointCheck()
            self.rate.sleep()


if __name__ == "__main__":
    LP = msgControl()
    LP.run()
