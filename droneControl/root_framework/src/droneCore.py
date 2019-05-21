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
from inspec_msg.msg import (pilot_cb)

mavros.set_namespace('mavros')
# mavros commands for 


onB_StateSub = '/onboard/state'
keySub = '/gcs/keypress'

linefollowsub = '/onboard/setpoint/linefollow'
homePubTopic = '/onboard/position/home'
wpDoneSub = '/onboard/check/WPSuccess'

homeReqPub = '/onboard/request/gpsHome'

pilotStateSub = '/onboard/check/pilotState'


class droneCore():
    def __init__(self):
        rospy.init_node('droneCMD_node')
        self.rate = rospy.Rate(20)
        self.first = True
        self.homeCoord = None
        self.gpsPos = NavSatFix()
        self.curPos = mavSP.PoseStamped()
        self.loiterPos = mavSP.PoseStamped()

        self.uavState = mavros_msgs.msg.State()
        self.setPoint = mavSP.PoseStamped()

        self.droneArmed = False
        self.isAirbourne = False
        self.sysState = None
            
        # Publishers/Subscribers
        self.statePub = rospy.Publisher(onB_StateSub, String, queue_size=1)
        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)

        self.homePub = rospy.Publisher(homePubTopic, mavros_msgs.msg.HomePosition, queue_size=1)
        self.messageHandlerPub = rospy.Publisher('/onboard/messageEnable', Bool, queue_size=1)

        # Services 
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

        rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self._cb_uavState)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self._cb_localPos)
        rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, self._cb_SatFix)
        rospy.Subscriber(mavros.get_topic('home_position', 'home'), mavros_msgs.msg.HomePosition, self._cb_HomeUpdate)
      
      
        rospy.Subscriber(keySub,Int8, self._cb_onKeypress)
        rospy.Subscriber(pilotStateSub, pilot_cb, self._pilotStateUpdate)
        rospy.Subscriber(homeReqPub, Bool, self.onHomeRequest)

        # rospy.Subscriber('/onboard/check/navComplete', Bool, self.onNavigationUpdate)

    # ROS-Specific functions     
    def _pubMsg(self,msg,topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()
        # print (msg)

    def _cb_uavState(self, msg):
        self.uavState = msg
        if not self.uavState.armed and self.sysState == 'loiter':
            self.statePub.publish('idle')
            pass
        pass

    def _cb_localPos(self, msg):
        self.curPos = msg
        if self.first:
            self.first = False
            if (self.uavState.armed == True) and (self.isAirbourne == False):
                if (self.curPos.pose.position.z > 1.0):
                    #if the node is started and the drone is airbourne, take control and loiter
                    self.isAirbourne = True
                    self.messageHandlerPub.publish(True)
                    self.setState('loiter')
                pass

    def _cb_SatFix(self,msg):
        self.gpsPos = msg

    def onHomeRequest(self, msg):
        if msg.data == True:
            self.homePub.publish(self.homeCoord)
            
    def _cb_HomeUpdate(self,msg):
        self.homeCoord = msg
    
    def _cb_onKeypress(self, msg):
        keypress = str(chr(msg.data))
        keypress.lower()
        if keypress == 't':
            self.droneTakeoff()
            if self.sysState == 'takeoff':
                self.droneLoiter()
        if keypress == 'l':
            if not self.isAirbourne:
                print ("Warn: Not airbourne!")
            else:
                self.droneLoiter()

        if keypress == 'm':
            if not self.isAirbourne:
                self.droneTakeoff(2.0)
                self.droneLoiter()
            self.missionEnable()
        
        if keypress == 'h':
            # self.homeCoord = self.gpsPos
            # self.homePub.publish(self.homeCoord)
            pass


    def _pilotStateUpdate(self,msg):
        if msg.complete == False:
            self.setState('loiter')
        else:
            pilot = msg.pilotName
            print("%s complete!" % pilot)
            if pilot == 'mission':
                self.setState('loiter')
            if pilot == 'takeoff':
                self.setMode()
            if pilot == 'landing':
                self.setMode('idle')

## Pilot Commands
    def setState(self, state):
        self.sysState = state
        self.statePub.publish(state)

    def droneTakeoff(self, alt=1.0):
        if self.isAirbourne == False or self.sysState == 'idle':
            if not self.droneArmed:
                mavCMD.arming(True)
            
            preArmMsgs = self.curPos
            preArmMsgs.pose.position.z = alt

            for i in range(50):
                self._pubMsg(preArmMsgs, self.spLocalPub)
            self.setState('takeoff')
            self.setMode(0,'OFFBOARD')
            self.setPoint = preArmMsgs

            self.isAirbourne = True
            self.messageHandlerPub.publish(self.isAirbourne)
            #wait until takeoff has occurred
            while(self.curPos.pose.position.z <= (preArmMsgs.pose.position.z-0.25)):
                self._pubMsg(self.setPoint, self.spLocalPub)
        else:
            self.setState('landing')
            self.setMode(0,'AUTO.LAND')
            while self.uavState.armed:
                self.rate.sleep()
            self.isAirbourne = False
            self.setState('idle')

    def droneLoiter(self):
        # print('loiter enable')
        self.setState('loiter')
        
    def missionEnable(self):
        self.setState('mission')
        pass
    
    def run(self):
        
        self.messageHandlerPub.publish(self.isAirbourne)
        while not rospy.is_shutdown():
            self.rate.sleep()
        pass


if __name__ == "__main__":
    uavCore = droneCore()
    uavCore.run()
    pass
