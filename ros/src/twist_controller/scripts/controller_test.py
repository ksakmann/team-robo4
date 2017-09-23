#!/usr/bin/env python

import math

import rospy
from twist_controller.msg import VelocitySetpoint
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from std_msgs.msg import Header


class ControllerTest(object):
    def __init__(self):
        rospy.init_node('controller_test')
        self.velocity_setpoint = 0.0
        self.yaw_rate_setpoint = 0.0
        self.setpoint_sub = rospy.Subscriber('/velocity_setpoint', VelocitySetpoint, self.setpoint_cb)
        self.cmd_pub = rospy.Publisher('/twist_cmd', TwistStamped, queue_size=10)
        self.loop()

    
    def loop(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            t = rospy.Time.now()
            # val = (5.0 * math.sin(1/5.0*2*math.pi*t.to_sec()) + 10) * 1.6 / 3.6
            # val = 10 * 1.6 / 3.6
            v_car = self.velocity_setpoint
            n_yaw = self.yaw_rate_setpoint * math.sin(1/5.0 * 2 * math.pi * t.to_sec())
            rospy.loginfo('Speed demand: %f Yaw rate demand: %f', v_car, n_yaw)

            vel_cmd = TwistStamped()
            vel_cmd.header = Header()
            vel_cmd.header.stamp = t
            vel_cmd.header.frame_id = '/vehicle'
            vel_cmd.twist.linear.x = v_car
            vel_cmd.twist.angular.z = n_yaw

            self.cmd_pub.publish(vel_cmd)

            rate.sleep()


    def setpoint_cb(self, msg):
        self.velocity_setpoint = msg.velocity
        self.yaw_rate_setpoint = msg.yaw_rate


# def run():
#     rospy.init_node('controller_test')
#     loop()

# def loop():

#     rate = rospy.Rate(50)

#     while not rospy.is_shutdown():
        
#         t = rospy.Time.now()
#         # val = (5.0 * math.sin(1/5.0*2*math.pi*t.to_sec()) + 10) * 1.6 / 3.6
#         # val = 10 * 1.6 / 3.6
#         rospy.loginfo('Speed demand: %f', val)

#         vel_cmd = TwistStamped()
#         vel_cmd.header = Header()
#         vel_cmd.header.stamp = t
#         vel_cmd.header.frame_id = '/vehicle'
#         vel_cmd.twist.linear.x = val

#         cmd_pub.publish(vel_cmd)

#         rate.sleep()


if __name__ == '__main__':
#     setpoint_sub = rospy.Subscriber('/velocity_cmd', VelocitySetpoint, self.speed_cb)
#     cmd_pub = rospy.Publisher('/twist_cmd', TwistStamped, queue_size=10)

    try:
        ControllerTest()
    except rospy.ROSInterruptException:
        pass