#!/usr/bin/env python

import rospy

import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP
import mavros.command as mavCMD

import mavros_msgs.msg as mavMSG
import mavros_msgs.srv

from std_msgs.msg import String

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'

descent_rate = 0.02 #m/s

class takeoffLandPilot():
    def __init__(self):
        rospy.init_node('takeoff_land_pilot')
        self.rate = rospy.Rate(20)



        rospy.Subscriber(mavros.get_topic('state'),
                         mavros_msgs.msg.State, self._cb_uavState)

        rospy.Subscriber(onB_StateSub, String, self._cb_onboard_StateChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                         mavSP.PoseStamped, self.onPositionChange)

        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)
        self.statePub = rospy.Publisher(onB_StateSub, String, queue_size=1)
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

        self.uavState = mavMSG.State()
        self.curPos = mavSP.PoseStamped()

        self.takeoff = False
        self.land = False

        
    def _pubMsg(self, msg, topic):
        msg.header = mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def _cb_onboard_StateChange(self, msg):     
        if msg.data == 'takeoff':
            print('takeoff enable')   
            self.takeoff = True
            self.land = False

        elif msg.data == 'land':
            print('landing enable')
            self.takeoff = False
            self.land = True
        else:
            self.takeoff = False
            self.land = False

    def _cb_uavState(self, msg):
        self.uavState = msg
        pass

    def onPositionChange(self, msg):
        self.curPos = msg
    
    def droneTakeoff(self, alt=3.0):
        if not self.uavState.armed:
            mavCMD.arming(True)

        preArmMsgs = self.curPos
        preArmMsgs.pose.position.z = alt
        for i in range(100):
            self._pubMsg(preArmMsgs, self.spLocalPub)

        self.setMode(0, 'OFFBOARD')
        self.setPoint = preArmMsgs

        #wait until takeoff has occurred
        while(self.curPos.pose.position.z <= (preArmMsgs.pose.position.z-0.25)):
            self._pubMsg(self.setPoint, self.spLocalPub)
        
        self.statePub.publish('loiter')
        

    def droneLand(self, descentRate=descent_rate):
        localPos = self.curPos
        while self.curPos.pose.position.z > 0.1:
            localPos.pose.position.z -= descent_rate
            self._pubMsg(localPos, self.spLocalPub)
            self.rate.sleep()
        rospy.sleep(1.0)
        mavCMD.arming(False)
        

    def run(self):

        while not rospy.is_shutdown():
            if self.takeoff:
                self.droneTakeoff()
           
            if self.land:
                self.droneLand()

if __name__ == "__main__":
    TLP = takeoffLandPilot()
    TLP.run()
