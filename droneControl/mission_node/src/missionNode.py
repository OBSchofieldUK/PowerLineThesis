#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import mavros 
import utm 
import mavros.command as mavCMD
import mavros.setpoint as mavSP

import mavros_msgs.msg
from std_msgs.msg import (String, Bool)
from sensor_msgs.msg import (NavSatFix)
from pylon_locator.msg import (pylonList, pylonDat)
mavros.set_namespace('mavros')

onB_StateSub    = '/onboard/state'
pylonRequest    = '/onboard/pylonRequest'
pylonListSub      = '/onboard/pylonList'

class missionPilot():
    def __init__(self):
        rospy.init_node('missionPilot')
        self.rate = rospy.Rate(20)
        self.enable = False

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(pylonListSub, pylonList, self.onPylonListUpdate)
        rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, self.cb_onPosUpdate)
        self.towerRequest = rospy.Publisher(pylonRequest, Bool, queue_size=1)

        self.dronePos = NavSatFix()
        self.pylonList = []
        self.received = False

# ROS Subscribers

    def onStateChange(self, msg):
        if msg.data == 'mission':
            print 'mission Enable'
            self.enable = True
            self.runMission()
        else:
            self.enable = False
        pass
    
    def onPylonListUpdate(self,msg):
        self.pylonList = msg.lists
        self.received = True
        pass

    def cb_onPosUpdate(self, msg):
        self.dronePos = msg


    def findNearestPylon(self): 
        utmDronePos = utm.from_latlon(self.dronePos.latitude, self.dronePos.longitude)
        nearestPylon = []
        closestDist = None
        for i in range(0,len(self.pylonList)):
            pylon = self.pylonList[i]
            pylonPos = utm.from_latlon(pylon.lat, pylon.lon)
            deltaNorthing = utmDronePos[0] - pylonPos[0]
            deltaEasting = utmDronePos[0] - pylonPos[0]
            dist = math.sqrt(deltaNorthing**2 + deltaEasting**2)

            if closestDist == None:
                closestDist = dist
                nearestPylon = self.pylonList[i]
            elif dist < closestDist:
                closestDist = dist
                nearestPylon = self.pylonList[i]

        print (nearestPylon)
        return nearestPylon

# Mission Pilot
    def runMission(self):
        self.towerRequest.publish(True)
        while not (self.received):
            self.rate.sleep()
        print(len(self.pylonList))
        if len(self.pylonList) > 0 :
            nextWP = self.findNearestPylon()

        pass


    def run(self):
        while not(rospy.is_shutdown()):
            self.rate.sleep()
        pass

if __name__ == "__main__":
    mp = missionPilot()
    mp.run()
