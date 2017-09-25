#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from std_msgs.msg import Header

def run():
    rospy.init_node('controller_test')
    loop()

def loop():

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        
        t = rospy.Time.now()
        # val = (5.0 * math.sin(1/5.0*2*math.pi*t.to_sec()) + 10) * 1.6 / 3.6
        val = 10 * 1.6 / 3.6
        rospy.loginfo('Speed demand: %f', val)

        vel_cmd = TwistStamped()
        vel_cmd.header = Header()
        vel_cmd.header.stamp = t
        vel_cmd.header.frame_id = '/vehicle'
        vel_cmd.twist.linear.x = val

        cmd_pub.publish(vel_cmd)

        rate.sleep()


if __name__ == '__main__':
    cmd_pub  = rospy.Publisher('/twist_cmd', TwistStamped, queue_size=10)

    try:
        run()
    except rospy.ROSInterruptException:
        pass