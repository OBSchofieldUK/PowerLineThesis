#!/usr/bin/env python
from math import (degrees, radians, floor, isinf)

import json
import rospy
import rospkg

from sensor_msgs.msg import LaserScan
from inspec_msg.msg import lidardat

lidarTopicSub = '/iris/laser/scan'
convLidarPub = '/inspec/daq/lidarDat'

class gazeboLeddarSim(object):
    def __init__(self):
        rospy.init_node('leddarsim_convert')
        self.getSettings()
        self.vu8Channels = []
        self.VU8Segments = 8

        rospy.Subscriber(lidarTopicSub, LaserScan, self.onLidarUpdate)
        self.lidarPub = rospy.Publisher(convLidarPub, lidardat, queue_size=1)
        self.rate = rospy.Rate(20)

    def getSettings(self):
        pathFind = rospkg.RosPack()
        packString = pathFind.get_path('inspec_lib')
        path = packString.replace('Library','settings.json')
        
        with open(path) as json_file:
            data = json.load(json_file)
            self.VU8Segments = data['Lidar']['Number of Segments']


    def onLidarUpdate(self, msg):
    
        inMsg = msg
        angleInc = degrees(inMsg.angle_increment)
        totalRange = int(degrees(inMsg.angle_max - inMsg.angle_min))

        inputSamples = len(inMsg.ranges)
        #print("Input samples: ", inputSamples)

        samplePerSeg = inputSamples/self.VU8Segments
        #print("Samples per segment: ", samplePerSeg)

        self.vu8Channels = []
        for seg in range(0, self.VU8Segments):
            nearestDist = 99999999.0
            for i in range(0, samplePerSeg):
                #print("Sample: ",i+seg*samplePerSeg)
                currSample = inMsg.ranges[i+seg*samplePerSeg]
                if not isinf(currSample):
                    if currSample < nearestDist:
                        nearestDist = currSample
            if nearestDist > 99999998.0:
                nearestDist = -1.0
            self.vu8Channels.append(nearestDist)

        outMsg = lidardat()
        outMsg.header.seq = msg.header.seq
        outMsg.header.stamp = msg.header.stamp
        outMsg.distance = self.vu8Channels
        self.lidarPub.publish(outMsg)
        pass

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    ls = gazeboLeddarSim()
    ls.run()
