#!/usr/bin/env python

import rospy
import rospkg
import time
from datetime import date

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

class getDroneState:
    def __init__(self):

        rospy.init_node('gazeboPos')
        self.filename = ""
        print("gazboPos ready")
        rospy.wait_for_service('/gazebo/get_model_state')
        self.modelCoord = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.posePub = rospy.Publisher('/dataLogger/gazebo_abs_pos_drone', PoseStamped, queue_size=0)
        # self.getSavePoint()
    
    def getSavePoint(self):
        rospack = rospkg.RosPack()
        absFile = rospack.get_path('data_logger')
        timeNow = time.ctime()
        logFilename = absFile+'Logs/'+'dataLog_'+timeNow+'.log'
        logFilename.replace(" ","_")
        timeNow.replace(' ', '_')
        print(timeNow)

    def run(self):
        try:
            while not rospy.is_shutdown():
                respCoords = self.modelCoord('irislidar_msc', '')
                outMsg = PoseStamped()
                outMsg.header = respCoords.header
                outMsg.pose = respCoords.pose
                self.posePub.publish(outMsg)
        except rospy.ServiceException as e:
            pass
if __name__ == "__main__":
    gDS = getDroneState()
    gDS.run()
