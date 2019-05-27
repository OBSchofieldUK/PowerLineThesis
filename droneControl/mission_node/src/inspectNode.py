#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt, radians, hypot
import rospy
import mavros
import utm
import mavros.command as mavCMD
import mavros.setpoint as mavSP

from mavros_msgs.msg import (AttitudeTarget)
from std_msgs.msg import (String, Bool, Header)
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from inspec_msg.msg import (pilot_cb, line_control_info)
mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
targetPub = '/onboard/setpoint/inspect'
powerlinePosSub = '/onboard/feedback/powerlinePosition'

startPos = mavSP.PoseStamped()

class inspectPilot():
    def __init__(self):
        rospy.init_node('inspectPilot')
        self.rate = rospy.Rate(20)
        self.enable = False
        self.pilotReady = False
        self.lineFound = False
        
        self.startpos = mavSP.PoseStamped()
        self.curLocalPos = None
        self.lineTarget = None
        self.targetPos = None

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position','pose'), mavSP.PoseStamped, self.onPosUpdate)
        rospy.Subscriber(powerlinePosSub, line_control_info, self.onLineUpdate)


        self.attiPub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        self.inpectPosSP = rospy.Publisher(targetPub, mavSP.PoseStamped, queue_size=1)

    # message handlers 
    def onPosUpdate(self, msg):
        self.curLocalPos = msg
        pass

    def onLineUpdate(self, msg):
        if self.lineFound==False:
            self.lineFound = True
        self.lineTarget = msg
    
    def _pubMsg(self, msg, topic):
        msg.header = mavros.setpoint.Header(
            frame_id="base_footprint",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def onStateChange(self, msg):
        if msg.data == 'inspect':
            self.enable = True
            print('inspection Enabled')
            self.lineTarget = None
            self.targetPos = self.curLocalPos
            
            self.lineFound = False


            # self.advanceToLine(self.curLocalPos)
            # self.pilotReady= True
            self.advanceToLine()
            self.alignToLine()
        else:
            if self.enable:
                print('inspection Disabled')
            self.enable = False
            self.pilotReady = False

    # Math Functions
    def calcDist2D(self, posA, posB):
        deltaX = posA.pose.position.x - posB.pose.position.x 
        deltaY = posA.pose.position.y - posB.pose.position.y
        dist = hypot(deltaX, deltaY)
        distAlt = sqrt(deltaX**2+deltaY**2)
        # print ("dX: %.2f, dY: %.2f, Dist: %.2f, Dist2: %.2f, " %(deltaX, deltaY, dist, distAlt))
        return deltaX, deltaY, dist

    # tester for attitude commands 
    def pubTestAtti(self):
        attiMsg = AttitudeTarget()
        attiMsg.body_rate = Vector3()
        attiMsg.header = Header()
        attiMsg.orientation = mavSP.Quaternion(
            *quaternion_from_euler(math.radians(-25), 0, 0))
        attiMsg.thrust = 0.71
        attiMsg.type_mask = 7  # ignore the body rate

        self._pubMsg(attiMsg, self.attiPub)

    # GPS inspection node: Wont work in real life, but good for proof of concept

    def advanceToLine(self):
        stepSize = 0.2
        step = self.curLocalPos
        step.pose.position.y += stepSize               
        # _,_,dist = self.calcDist2D(self.curLocalPos, step)
        # print(dist)
        numSteps=int(20/stepSize)
        self.targetPos = step
        self.pilotReady = True
        print("starting search")

        for i in range(1, numSteps):
            step.pose.position.y += stepSize
            # _, _, dist = self.calcDist2D(self.curLocalPos, step)
            d = rospy.Duration(0.2)
            rospy.sleep(d)
            print(i)
            self.targetPos = step
            if self.lineFound:
                break

        if not self.lineFound:
            print("WARN: line not found :( ")
        else:
            print("line found!")
            self.targetPos.pose.position.y += 0.5

    def alignToLine(self):
        pitchGain = 0.1
        print("Line Alignment")
        if self.targetPos == None:
            self.targetPos = self.curLocalPos

        if self.lineFound and self.lineTarget.trusted:
            yaw = self.lineTarget.Yaw
            pitchError = self.lineTarget.y
            if yaw < 0:
                self.targetPos.pose.orientation = Quaternion(*quaternion_from_euler(0,0,radians(yaw)))
            else:
                self.targetPos.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(-yaw)))

            if abs(pitchError) > 0.5:
                pitchGain = 0.15
            else: 
                pitchGain = 0.1
            timeout = rospy.Time.now()
            count = 0
            timeoutCheck = True
            while (timeout - rospy.Time.now()) < rospy.Duration(30):
                pitchError = self.lineTarget.y
                if abs(pitchError) > 0.5:
                    pitchGain = 0.15
                else:
                    pitchGain = 0.1
                
                d = rospy.Duration(0.2) 
                rospy.sleep(d)
                
                self.targetPos.pose.position.y -= pitchError * pitchGain

                if abs(pitchError) < 0.075:
                    count +=1
                    # print(count)
                if count > 25:
                    timeoutCheck = False
                    break
            if timeoutCheck:
                print("timeout")

    def startLanding(self):
        # global startPos
        self.lineFound = False

        print('pause')
        d = rospy.Duration(2.0)
        rospy.sleep(d)
        curPos = self.curLocalPos

        self.pilotReady = True

  
        print('pilotReady')
        #step 1, advance towards line 
        startingPosition = curPos

        target = mavSP.PoseStamped()
        target.pose.position = startingPosition.pose.position
        target.pose.orientation = curPos.pose.orientation 
        target.pose.position.y += 0.25
        # _,_,distTravelled = self.calcDist2D(startingPosition, target)
        self.targetPos = target
        distTravelled = 0

        while (distTravelled < 15):

            # print("Start:", startingPosition.pose)
            # print("Targ:", self.targetPos)

            self.targetPos.pose.position.y += 0.2
            
            d = rospy.Duration(0.2)
            rospy.sleep(d)
            
            # _,_,distTravelled = self.calcDist2D(startingPosition, self.targetPos)

            if self.lineFound:
                print("lineFound!")
                self.targetPos.pose.position.y += 0.5
                break        
        
        #step2, when detected, align yaw (+xy)
        count = 0
        while(self.enable==True):
            
            self.targetPos.pose.position.y -= (self.lineTarget.y * 0.1)
            # if self.lineTarget:
            self.targetPos.pose.orientation = Quaternion(*quaternion_from_euler(0,0, radians(-self.lineTarget.Yaw)))
            if (self.lineTarget.y < 0.05):
                count += 1
            d = rospy.Duration(0.2)
            rospy.sleep(d)

            if self.enable != True or count > 40:
                break

        #step3, after timePeriod, advance up to specified height

        while(self.lineTarget.z > 0.15):
            if self.lineTarget.z > 2:
                self.targetPos.pose.position.y -= (self.lineTarget.y * 0.15)
                if self.lineTarget.y < 0.1:
                    self.targetPos.pose.position.z += 0.1
            elif self.lineTarget.z < 1:
                self.targetPos.pose.position.y -= (self.lineTarget.y * 0.1)
                if self.lineTarget.y < 0.05:
                    self.targetPos.pose.position.z += 0.05
            else:
                if self.lineTarget.z < 0.5:
                    self.targetPos.pose.position.y -= (self.lineTarget.y * 0.1)
                    if self.lineTarget.y < 0.025:
                        self.targetPos.pose.position.z += 0.015
            print self.targetPos.pose.position.z, self.targetPos.pose.position.y

            d = rospy.Duration(0.2)
            rospy.sleep(d)
        pass
        print("done")
        
    def run(self):
        while not(rospy.is_shutdown()):
            if self.enable:
                if self.pilotReady:
                    self._pubMsg(self.targetPos, self.inpectPosSP)
            else:
                if self.curLocalPos != None:
                    self._pubMsg(self.curLocalPos, self.inpectPosSP)
            self.rate.sleep()
        pass

if __name__ == "__main__":
    ip = inspectPilot()
    ip.run()
