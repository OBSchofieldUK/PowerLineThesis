#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import mavros
import utm
import mavros.command as mavCMD
import mavros.setpoint as mavSP

from mavros_msgs.msg import (AttitudeTarget)
from std_msgs.msg import (String, Bool, Header)
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
targetPub = '/onboard/setpoint/inspection'


class inspectPilot():
    def __init__(self):
        rospy.init_node('inspectPilot')
        self.rate = rospy.Rate(20)
        self.enable = False
        self.count = 0
        self.curLocalPos = None

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self.onPosUpdate)
        
        self.attiPub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        
    def onPosUpdate(self, msg):
        self.curLocalPos = msg
        pass
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
        else:
            if self.enable:
                print('inspection Disabled')
            self.enable = False

    def pubTestAtti(self):
        attiMsg = AttitudeTarget()
        attiMsg.body_rate = Vector3()
        attiMsg.header = Header()
        attiMsg.orientation = mavSP.Quaternion(*quaternion_from_euler(0, math.radians(25), 0 ))
        attiMsg.thrust = 0.7
        attiMsg.type_mask = 7 #ignore the body rate

        self._pubMsg(attiMsg, self.attiPub)
        
    def run(self):
        while not(rospy.is_shutdown()):
            if self.enable:
                self.pubTestAtti()
                pass
            self.rate.sleep()
        pass


if __name__ == "__main__":
    ip = inspectPilot()
    ip.run()


# class MavrosOffboardAttctlTest(MavrosTestCommon):
#     """
#     Tests flying in offboard control by sending attitude and thrust setpoints
#     via MAVROS.
#     For the test to be successful it needs to cross a certain boundary in time.
#     """

#     def setUp(self):
#         super(MavrosOffboardAttctlTest, self).setUp()

#         self.att = AttitudeTarget()

#         self.att_setpoint_pub = rospy.Publisher(
#             'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

#         # send setpoints in seperate thread to better prevent failsafe
#         self.att_thread = Thread(target=self.send_att, args=())
#         self.att_thread.daemon = True
#         self.att_thread.start()

#     def tearDown(self):
#         super(MavrosOffboardAttctlTest, self).tearDown()

#     #
#     # Helper methods
#     #
#     def send_att(self):
#         rate = rospy.Rate(10)  # Hz
#         self.att.body_rate = Vector3()
#         self.att.header = Header()
#         self.att.header.frame_id = "base_footprint"
#         self.att.orientation = Quaternion(*quaternion_from_euler(-0.25, 0.5,
#                                                                  0))
#         self.att.thrust = 0.7
#         self.att.type_mask = 7  # ignore body rate

#         while not rospy.is_shutdown():
#             self.att.header.stamp = rospy.Time.now()
#             self.att_setpoint_pub.publish(self.att)
#             try:  # prevent garbage in console output when thread is killed
#                 rate.sleep()
#             except rospy.ROSInterruptException:
#                 pass

#     #
#     # Test method
#     #
#     def test_attctl(self):
#         """Test offboard attitude control"""
#         # boundary to cross
#         boundary_x = 200
#         boundary_y = 100
#         boundary_z = 50

#         # make sure the simulation is ready to start the mission
#         self.wait_for_topics(60)
#         self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
#                                    10, -1)

#         self.log_topic_vars()
#         self.set_mode("OFFBOARD", 5)
#         self.set_arm(True, 5)

#         rospy.loginfo("run mission")
#         rospy.loginfo("attempting to cross boundary | x: {0}, y: {1}, z: {2}".
#                       format(boundary_x, boundary_y, boundary_z))
#         # does it cross expected boundaries in 'timeout' seconds?
#         timeout = 90  # (int) seconds
#         loop_freq = 2  # Hz
#         rate = rospy.Rate(loop_freq)
#         crossed = False
#         for i in xrange(timeout * loop_freq):
#             if (self.local_position.pose.position.x > boundary_x and
#                     self.local_position.pose.position.y > boundary_y and
#                     self.local_position.pose.position.z > boundary_z):
#                 rospy.loginfo("boundary crossed | seconds: {0} of {1}".format(
#                     i / loop_freq, timeout))
#                 crossed = True
#                 break

#             try:
#                 rate.sleep()
#             except rospy.ROSException as e:
#                 self.fail(e)

#         self.assertTrue(crossed, (
#             "took too long to cross boundaries | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
#             format(self.local_position.pose.position.x,
#                    self.local_position.pose.position.y,
#                    self.local_position.pose.position.z, timeout)))

#         self.set_mode("AUTO.LAND", 5)
#         self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
#                                    90, 0)
#         self.set_arm(False, 5)


# if __name__ == '__main__':
#     import rostest
#     rospy.init_node('test_node', anonymous=True)

#     rostest.rosrun(PKG, 'mavros_offboard_attctl_test',
#                    MavrosOffboardAttctlTest)
