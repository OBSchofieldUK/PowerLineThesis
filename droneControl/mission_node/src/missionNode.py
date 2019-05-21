#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import mavros 
import utm 
import mavros.command as mavCMD
import mavros.setpoint as mavSP

import mavros_msgs.msg
from std_msgs.msg import (String, Bool, Int8)
from sensor_msgs.msg import (NavSatFix)
# from pylon_locator.msg import (pylonList, pylonDat)
from inspec_msg.msg import (pilot_cb, pylonList, pylonDat)

mavros.set_namespace('mavros')

onB_StateSub    = '/onboard/state'
pylonRequest    = '/onboard/pylonRequest'
pylonListSub      = '/onboard/pylonList'

targetWP = '/onboard/setpoint/mission'
wpComplete = '/onboard/check/WPSuccess'

pilotStatePub = '/onboard/check/pilotState'
class missionPilot():
    def __init__(self):
        rospy.init_node('missionPilot')
        self.rate = rospy.Rate(20)
        self.enable = False

        self.dronePos = NavSatFix()
        self.pylonList = []
        self.received = False
        self.pylonOffset = 25 # meters away from the power pylon 
        self.coordSent = False

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(pylonListSub, pylonList, self.onPylonListUpdate)
        rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, self.cb_onPosUpdate)
        rospy.Subscriber(wpComplete, Int8, self.onWPArrival)

        self.wpPub = rospy.Publisher(targetWP, NavSatFix, queue_size=1)
        self.towerRequest = rospy.Publisher(pylonRequest, Bool, queue_size=1)
        self.pilotStatePub = rospy.Publisher(pilotStatePub, pilot_cb, queue_size=1)

    def sendState(self, state):
        psMsg = pilot_cb()
        psMsg.pilotName = 'mission'
        psMsg.complete = state
        self.pilotStatePub.publish(psMsg)
        
        # ROS Subscribers
    def onStateChange(self, msg):
        if msg.data == 'mission':
            print ('mission Enable')
            self.enable = True
            self.runMission()
        else:
            print('mission Disable')
            self.enable = False
        pass
    

    def onPylonListUpdate(self,msg):
        self.pylonList = msg.lists
        self.received = True
        pass

    def cb_onPosUpdate(self, msg):
        self.dronePos = msg

    def onWPArrival(self, msg):
        if msg.data == 2 and self.coordSent:
            # print("At Desination!")
            self.sendState(True)
        pass
    def runMission(self):
        self.coordSent = False
        self.navToPylon()
    '''
    Calculations for 
        - finding nearest pylon to a reference point (input UTM Coords)
    '''
    def findNearestPylon(self, refUTM):

        nearestPylon = []
        closestDist = None
        pylonidx = -1
        for i in range(0, len(self.pylonList)):

            pylonPos = utm.from_latlon(
                self.pylonList[i].lat, self.pylonList[i].lon)
            # deltaNorthing = utmDronePos[0] - pylonPos[0]
            # deltaEasting = utmDronePos[0] - pylonPos[0]
            # dist = math.sqrt(deltaNorthing**2 + deltaEasting**2)
            _, _, dist = self.calcDist(refUTM, pylonPos)
            if dist > 0:
                if closestDist == None:
                    closestDist = dist
                    nearestPylon = self.pylonList[i]
                    pylonidx = i
                elif dist < closestDist:
                    closestDist = dist
                    nearestPylon = self.pylonList[i]
                    pylonidx = i
        # print("Nearest Pylon is at:", nearestPylon, " Dist:", closestDist)

        return pylonidx, nearestPylon

    def calcDist(self, utmPosA, utmPosB):
        dist = -1
        deltaEast = utmPosA[0]-utmPosB[0]
        deltaNorth = utmPosA[1]-utmPosB[1]
        if deltaNorth !=0:
            dist = math.sqrt(deltaNorth**2 + deltaEast**2)
        return deltaNorth, deltaEast, dist
 
    def calcOrientation(self, pylonidx):
        targUTM = utm.from_latlon(self.pylonList[pylonidx].lat, self.pylonList[pylonidx].lon)
        _, adjPylon = self.findNearestPylon(targUTM)

        nxtUTM = utm.from_latlon(adjPylon.lat, adjPylon.lon)
        northing, easting,_ = self.calcDist(targUTM, nxtUTM)
        orientation = math.degrees(math.atan(easting/northing))
        offsetNorth = -self.pylonOffset*math.cos(180-(orientation))
        offsetEast = self.pylonOffset*math.sin(180-(orientation))
        
        # print (orientation, offsetNorth, offsetEast)
        return orientation, offsetNorth, offsetEast

    # Mission Pilot
    def navToPylon(self):
        self.towerRequest.publish(True)

        while not (self.received):
            self.rate.sleep()
        
        if len(self.pylonList) > 0 :
            utmDronePos = utm.from_latlon(self.dronePos.latitude, self.dronePos.longitude)  
            nearidx, nextWP = self.findNearestPylon(utmDronePos)
            _, wpOffN, wpOffE = self.calcOrientation(nearidx)
            
            utmNxtWP = utm.from_latlon(nextWP.lat, nextWP.lon)
            nxtWPAdjN = utmNxtWP[0] + wpOffN
            nxtWPAdjE = utmNxtWP[1] + wpOffE
            WPADJ = utm.to_latlon(nxtWPAdjN, nxtWPAdjE, utmNxtWP[2], utmNxtWP[3])
            print("Waypoint to pylon is:", WPADJ)
            wpTarget = NavSatFix()
            wpTarget.latitude = WPADJ[0]
            wpTarget.longitude = WPADJ[1]
            wpTarget.altitude = 10.0
            self.wpPub.publish(wpTarget)
            print("coordSent")

            self.coordSent = True

        else:
            print("Warning: no pylons found! loitering...")
            self.sendState(False)
        
    def run(self):
        while not(rospy.is_shutdown()):
            if self.enable:
                
                pass
            self.rate.sleep()
        pass

if __name__ == "__main__":
    mp = missionPilot()
    mp.run()

