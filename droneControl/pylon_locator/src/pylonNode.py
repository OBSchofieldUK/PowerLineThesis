#!/usr/bin/env python
# -*- coding: utf-8 -*-


import utm 
import rospy
import mavros

import mavros.setpoint as mavSP


from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix

from pylon_locator.msg import (pylonDat, pylonList) 

import requests 
from xml.etree import ElementTree

requestSub = '/onboard/pylonRequest'
pylonDataPub = '/onboard/pylonList'

mavros.set_namespace('mavros')

class pylon_node():
    def __init__(self):
        rospy.init_node('pylon_node')
        rospy.Subscriber(requestSub, Bool, self.cb_onRequest)
        rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, self.cb_onPosUpdate)
        self. pylonPub = rospy.Publisher(pylonDataPub, pylonList, queue_size=1)
        self.osmAPI = "https://api.openstreetmap.org/api/0.6/map"
        self.boundary = 150 #metres -- curpos +/- (boundary / 2)  
        self.dronePos = NavSatFix()
        self.pylonList = []
        rospy.spin()
    pass

    def cb_onPosUpdate(self, msg):
        self.dronePos = msg

    def cb_onRequest(self, msg):
        
        if msg.data == True:
            if self.dronePos.latitude == 0.0 or self.dronePos.longitude==0.0:
                rospy.logwarn("Warning: unable to get drone coordinates!")
            else:
                boundInter = (self.boundary/2) 
                curLatlon = [self.dronePos.latitude, self.dronePos.longitude]
                utmPos = utm.from_latlon(curLatlon[0], curLatlon[1])
                topleftBound = [utmPos[0] - boundInter, utmPos[1] - boundInter ]
                bottomRightBound = [utmPos[0] + boundInter, utmPos[1] + boundInter]

                boundTL = utm.to_latlon(topleftBound[0], topleftBound[1], utmPos[2], utmPos[3])
                boundBR = utm.to_latlon(bottomRightBound[0], bottomRightBound[1], utmPos[2], utmPos[3])

                boundbox = str(boundTL[1])+","+str(boundTL[0])+","+str(boundBR[1])+","+str(boundBR[0])
                # print boundbox
                self.getPylons(boundbox)
                self.pylonPub.publish(self.pylonList)


    def getPylons(self, boundaryStr):
        payload = {"bbox": boundaryStr}
        r = requests.get(url=self.osmAPI, params=payload)
        
        xmlRoot = ElementTree.fromstring(r.content)

        # working find power nodes
        nodes = xmlRoot.findall('node')
        numTowers = xmlRoot.findall("node//tag/[@v='tower']")
        # print("number of towers: ", len(numTowers))
        towerList = []
        for child in nodes:
            tags = child.findall(".//tag/[@k]")
            towerType = ""
            for element in tags:
                kVal = element.get('k')
                vVal = element.get('v')
                if kVal == 'design':
                    towerType = vVal

                elif (kVal == 'power' and vVal == 'tower'):
                    towerLat = child.get('lat')
                    towerLon = child.get('lon')
                    pylon = pylonDat()
                    pylon.lat = float(towerLat)
                    pylon.lon = float(towerLon)
                    pylon.type = towerType
                    towerList.append(pylon)
                    towerType = ""

                    # print pylon.lat, '\t', pylon.lon

        self.pylonList = towerList

        return towerList

if __name__ == "__main__":
    pl = pylon_node()
    
    pass
 
