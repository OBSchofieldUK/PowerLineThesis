#!/usr/bin/env python
# -*- coding: utf-8 -*-

import datetime

from math import sqrt, radians, hypot, degrees, pi
import rospy
import rospkg
import mavros
import utm
import mavros.command as mavCMD
import mavros.setpoint as mavSP

from mavros_msgs.msg import (AttitudeTarget)
from std_msgs.msg import (String, Bool, Header, Int8)
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from inspec_msg.msg import (pilot_cb, line_control_info)



mavros.set_namespace('mavros')
onB_StateSub = '/onboard/state'
targetPub = '/onboard/setpoint/inspect'
powerlinePosSub = '/onboard/feedback/powerlinePosition'
pilotStatePub = '/onboard/check/pilotState'

class inspectPilot():
    def __init__(self):
        self.DEBUG = True

        rospy.init_node('inspectPilot')
        self.rate = rospy.Rate(20)
        self.enable = False
        self.pilotReady = False
        self.lineFound = False
        
        self.startpos = mavSP.PoseStamped()
        self.curLocalPos = None
        self.lineTarget = None
        self.targetPos = None
        self.wpStatus = -1

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position','pose'), mavSP.PoseStamped, self.onPosUpdate)
        rospy.Subscriber(powerlinePosSub, line_control_info, self.onLineUpdate)
        rospy.Subscriber('/onboard/check/WPSuccess', Int8, self.onWPUpdate)
        self.pilotStatePub = rospy.Publisher(pilotStatePub, pilot_cb, queue_size=1)
        self.attiPub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        self.inpectPosSP = rospy.Publisher(targetPub, mavSP.PoseStamped, queue_size=1)
        if self.DEBUG:
            self.timeEnabled = None
            print("InspectNode: Debug Enabled")
            # self.filename = self.getFilename()
            # print('(DEBUG) writing to: %s' % self.filename)

    def getFilename(self):
        filename = 'droneCTRL_'+datetime.datetime.now().strftime('%d%m%y_%H:%M')+'.csv'
        rpkg = rospkg.RosPack()
        path = rpkg.get_path('root_framework').rsplit('/',1)[0]        
        concatFile = path+'/logs/'+filename
        #             write_to_logfile("Writing data to '%s'\r\n" % filename)
                    
        fa = open(concatFile,'a+')
        fa.write("Time, State, posX, posY, posZ, errorX, errorY, errorZ, errorYaw, ascendRate, adjustmentGain, delaytime\r\n")
        fa.close()

        return concatFile

    def writeLog(self,state, ascendRate, adjGain, delayTime):
        logFile = self.filename
        uavPos = self.curLocalPos.pose.position
        errorPos = self.lineTarget
        timeElapsed = rospy.Time.now() - self.timeEnabled
        if errorPos == None:
            errorPos = line_control_info()
            errorPos.x = -1.0
            errorPos.y = -1.0
            errorPos.z = -1.0
            errorPos.Yaw = 0.0

        fa = open(logFile, 'a+')
        # fa.write(
            # "Time, posX, posY, posZ, errorX, errorY, errorZ, errorYaw, ascendRate, adjustmentGain, delaytime\r\n")
        output = "%.2f, %i,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f,%.3f,%.3f\r\n" % (timeElapsed.to_sec(), state, uavPos.x, uavPos.y, uavPos.z, errorPos.x, errorPos.y, errorPos.z, degrees(errorPos.Yaw), ascendRate, adjGain, delayTime)
        fa.write(output)
        fa.close()


    # message handlers 
    def onPosUpdate(self, msg):
        self.curLocalPos = msg
        pass

    def sendState(self, state):
        psMsg = pilot_cb()
        psMsg.pilotName = 'inspect'
        psMsg.complete = state
        self.pilotStatePub.publish(psMsg)

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
            # self.targetPos = mavSP.PoseStamped()
            print('inspection Enabled')
            self.lineTarget = None
            self.targetPos = None
            self.lineFound = False
            self.filename = self.getFilename()
            print('(DEBUG) writing to: %s' % self.filename)
            
            self.timeEnabled = rospy.Time.now()
            
            self.pilotReady= True
            self.reduceAltitude()
            self.advanceToLine()
            if self.lineFound:
                self.alignToLine()
                self.ascendToLine()
        else:
            if self.enable:
                print('inspection Disabled')
            self.enable = False
            self.pilotReady = False

    def onWPUpdate(self,msg):
        self.wpStatus = msg.data
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
    def reduceAltitude(self):
        self.targetPos = mavSP.PoseStamped()
        self.targetPos.pose.position.x = self.curLocalPos.pose.position.x
        self.targetPos.pose.position.y = self.curLocalPos.pose.position.y
        self.targetPos.pose.position.z = 7.5
        self.targetPos.pose.orientation = self.curLocalPos.pose.orientation
        targAlt = self.targetPos.pose.position.z
        curAlt = self.curLocalPos.pose.position.z
        # print(targAlt, curAlt, targAlt-curAlt)
        while( abs(targAlt-curAlt) > 0.5):
            deltaAlt = abs(targAlt-curAlt)
            # print(deltaAlt)
            self.rate.sleep()
            curAlt = self.curLocalPos.pose.position.z
    def advanceToLine(self):
        stepSize = 0.2
        step = self.curLocalPos
        step.pose.position.y += stepSize               
        # _,_,dist = self.calcDist2D(self.curLocalPos, step)
        # print(dist)
        numSteps=int(25/stepSize)
        self.targetPos = step
        self.pilotReady = True
        print("starting search")

        for i in range(1, numSteps):
            step.pose.position.y += stepSize
            # _, _, dist = self.calcDist2D(self.curLocalPos, step)
            if i > (numSteps/2):
                d = rospy.Duration(0.25)
            else:
                d = rospy.Duration(0.2)
            rospy.sleep(d)
            # print(i)
            self.targetPos = step
            self.writeLog(0,0.0,0.0,d.to_sec())
            if self.lineFound:
                break
            if not self.enable:
                break 

        if self.enable:
            if not self.lineFound:
                print("WARN: line not found :( ")
            else:
                print("line found!")
                self.targetPos.pose.position.y += 0.5
        else:
            self.targetPos = None


    def alignToLine(self):
        pitchGain = 0.1
        print("Line Alignment")
        if self.targetPos == None:
            self.targetPos = self.curLocalPos

        if self.lineFound and self.lineTarget.trusted:
            yawError = self.lineTarget.Yaw
            pitchError = self.lineTarget.y
            # print("YawError", yawError)
            if yawError < 0:
                # print("YawPlus")
                yaw = pi + yawError
            else:
                # print("Yaw")
                yaw = yawError
            # print (yaw)
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
                if abs(pitchError) > 0.1: 
                    pitchGain = 0.15 #induce wobble?
                else:
                    pitchGain = 0.1
                
                d = rospy.Duration(0.2) 
                rospy.sleep(d)
                
                self.targetPos.pose.position.y -= pitchError * pitchGain
                self.writeLog(1, 0.0, pitchGain, d.to_sec())
                if abs(pitchError) < 0.075:
                    count +=1
                    # print(count)
                if count > 25:
                    timeoutCheck = False
                    break
                if self.enable == False:
                    break

            if timeoutCheck:
                print("timeout")
            else: 
                # print("ready To Ascend")
                pass

    def ascendToLine(self):
        timeout = rospy.Time.now()
        toCheck = True
        ascendRate = 0.1
        count = 0
        print('Ascend to line')
        countTime = 0.0
        while(True):
            curTime = rospy.Time.now()-timeout
            ascendRate = 0
            adjustGain = 0.01
            delay = 0.1
            
            if self.lineTarget.z > 0.3:
                ascendRate = 0.01
                adjustGain = 0.005
                delay = 0.15

            if self.lineTarget.z > 1.0:
                ascendRate = 0.025
                adjustGain = 0.05
                delay = 0.25

            if self.lineTarget.z > 2.5:
                ascendRate = 0.15
                adjustGain = 0.1
                delay = 0.2

            groundDist = 14.5 - self.curLocalPos.pose.position.z
            self.targetPos.pose.position.y += -(self.lineTarget.y * adjustGain)
            self.targetPos.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(self.lineTarget.Yaw)))

            if abs(self.lineTarget.y) < 0.05:

                self.targetPos.pose.position.z += ascendRate
                
                if self.lineTarget.z < 0.25:
                    # print(count)
                    count += 1
                    countTime = rospy.Time.now()
            
            # if count > 1:
            #     countTimeout = rospy.Time.now()-countTime
            #     if countTimeout > rospy.Duration(5.0):      #assume estimator has lost track, reduce and reset. 
            #         self.targetPos.pose.position.z -= 2.5
            #         count = 0
            
            d = rospy.Duration(delay)
            rospy.sleep(d)
            
            if count > 15:
                toCheck = False
                break
            if curTime > rospy.Duration(120):
                break
            if self.DEBUG:
                print("%.2f \t actDist: %.2f, estDist: %.2f, \t ascRate: %.3f, adjGain: %.3f, count: %d" % (curTime.to_sec(), groundDist, self.lineTarget.z, ascendRate, adjustGain, count))
                self.writeLog(2, ascendRate, adjustGain, delay)

            if self.enable == False:
                    break
        if self.enable:
            if toCheck:
                print("timeout!")
                d = rospy.Duration(5.0)
                rospy.sleep(d)
                self.targetPos.pose.position.z -= 2.0
                
                self.sendState(False)

            else:
                print("done")
                self.sendState(True)      

    ## OBSOLETE FUNCTION #TODO: Delete startLanding()
    def startLanding(self):
        # global startPos
        self.lineFound = False
        self.targetPos = self.curLocalPos
        self.targetPos.pose.position.z = 7.5
        self._pubMsg(self.targetPos, self.inpectPosSP)
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
        target.pose.position.z = 7.5

        target.pose.position.y += 0.25
        # _,_,distTravelled = self.calcDist2D(startingPosition, target)
        self.targetPos = target
        distTravelled = 0
        stepincrease = 0.2
        while (distTravelled < 15):

            # print("Start:", startingPosition.pose)
            # print("Targ:", self.targetPos)

            self.targetPos.pose.position.y += stepincrease
            
            d = rospy.Duration(0.2)
            rospy.sleep(d)
            
            # _,_,distTravelled = self.calcDist2D(startingPosition, self.targetPos)

            if self.lineFound:
                print("lineFound!")
                self.targetPos.pose.position.y += 0.5
                break        
            distTravelled += stepincrease

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
