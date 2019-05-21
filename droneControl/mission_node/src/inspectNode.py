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

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
pylonRequest = '/onboard/pylonRequest'
pylonListSub = '/onboard/pylonList'

targetWP = '/onboard/setpoint/mission'


class missionPilot():
    def __init__(self):
        rospy.init_node('missionPilot')
        self.rate = rospy.Rate(20)
        self.enable = False

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(pylonListSub, pylonList, self.onPylonListUpdate)
        rospy.Subscriber(mavros.get_topic('global_position',
                                          'global'), NavSatFix, self.cb_onPosUpdate)
        self.wpPub = rospy.Publisher(targetWP, NavSatFix, queue_size=1)

        self.towerRequest = rospy.Publisher(pylonRequest, Bool, queue_size=1)



    # ROS Subscribers
    def onStateChange(self, msg):
        if msg.data == 'inspection':
            print ('inspection enabled')
            self.enable = True
            self.runMission()
        else:
            self.enable = False
        pass

    def onPylonListUpdate(self, msg):
        self.pylonList = msg.lists
        self.received = True
        pass

    def cb_onPosUpdate(self, msg):
        self.dronePos = msg

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


        # print (orientation, offsetNorth, offsetEast)
        return orientation, offsetNorth, offsetEast

    # Mission Pilot

    def runMission(self):
        self.navToPylon()

    def run(self):
        while not(rospy.is_shutdown()):
            if self.enable:

                pass
            self.rate.sleep()
        pass


if __name__ == "__main__":
    mp = missionPilot()
    mp.run()
