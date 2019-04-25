#!/usr/bin/env python

import rospy
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command

from mavros_msgs.msg import GlobalPositionTarget
import mavros_msgs.srv
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import String

mavros.set_namespace('mavros')

state_sub = '/gcs/state'
curPos_sub = mavros.get_topic('global_position', 'global')
loiter_Pub = mavros.get_topic('setpoint_raw', 'global')
uavState = mavros_msgs.msg.State()


def _state_callback(topic):
    uavState.armed = topic.armed
    uavState.connected = topic.connected
    uavState.mode = topic.mode
    uavState.guided = topic.guided


class loiterPilot(object):

    def __init__(self):
        rospy.init_node('loiterPilot')
        self.rate = rospy.Rate(20)
        self.Enable = False;

        rospy.Subscriber(state_sub, String, self._onStateUpdate)
        # rospy.Subscriber(curPos_sub, NavSatFix, self._onPosUpdate)

        # locPosSub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, _local_pos_callback)
        self.locPosSub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self._local_pos_callback)
        loiterPub = mavros.setpoint.get_pub_position_local(queue_size=5)

        self.setPointMsg = SP.PoseStamped()

    def _local_pos_callback(self, msg):
        localSP = SP.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id='att_pose',
                stamp=rospy.Time.now()),
        )

        localSP.pose.position.x = msg.pose.position.x
        localSP.pose.position.y = msg.pose.position.y
        localSP.pose.position.z = msg.pose.position.z

        if not self.Enable:
            self.setPointMsg = localSP

    def _onStateUpdate(self, msg):
        if (msg.data=='loiter'):
            self.Enable = True
            print('loiter enable.')
        else:
            self.Enable = False

        pass

    def run(self):
        print('running...')
        while not rospy.is_shutdown():
            # print(self.curPosMsg)
            if self.Enable == True:
                self.loiterPub.publish(self.setPointMsg)
                print('publishing')
            rospy.spin()


if __name__ == '__main__':
    lp = loiterPilot()
    lp.run()
