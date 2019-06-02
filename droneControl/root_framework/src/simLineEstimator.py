#!/usr/bin/env python

import rospy
import rospkg
import time
from datetime import date

from math import radians

from inspec_msg.msg import line_control_info
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import (PoseStamped, Quarternion)

class getDroneState:
    def __init__(self):

        rospy.init_node('line_estimator')
        self.filename = ""
        print("gazboPos ready")
        rospy.wait_for_service('/gazebo/get_model_state')
        self.modelCoord = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.posePub = rospy.Publisher('/onboard/feedback/powerlinePosition', line_control_info, queue_size=1)
        
        self.powerLinePos = [0, 20, 14.5] #powerline position
        # self.getSavePoint()
    
    def getSavePoint(self):
        rospack = rospkg.RosPack()
        absFile = rospack.get_path('data_logger')
        timeNow = time.ctime()
        logFilename = absFile+'Logs/'+'dataLog_'+timeNow+'.log'
        logFilename.replace(" ","_")
        timeNow.replace(' ', '_')
        print(timeNow)

    def formatEstimator(self):
        respCoords = self.modelCoord('irislidar_msc', '')
        outMsg = line_control_info()

        if abs(self.powerLinePos[1] - respCoords.pose.position.y) < 5: #only publish when within 5Metres of the line 
            outMsg.x = self.powerLinePos[0]
            outMsg.y = self.powerLinePos[1] -respCoords.pose.position.y
            outMsg.z = self.powerLinePos[2] - respCoords.pose.position.z
            _,_,yawPos = euler_from_quaternion(Quarternion(*respCoords.pose.position))
            outMsg.yaw = Quarternion(*quaternion_from_euler(0,0,radians(90-yawPos)))
            print(outMsg)
            self.posePub.publish(outMsg)
        pass

    def run(self):
        try:
            while not rospy.is_shutdown():
                # self.formatEstimator()
                # respCoords = self.modelCoord('irislidar_msc', '')

                # outMsg = PoseStamped()
                # outMsg.header = respCoords.header
                # outMsg.pose = respCoords.pose
                # self.posePub.publish(outMsg)
                self.formatEstimator()
        except rospy.ServiceException as e:
            pass
if __name__ == "__main__":
    gDS = getDroneState()
    gDS.run()
