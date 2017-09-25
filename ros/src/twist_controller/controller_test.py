import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped


def run():
    rospy.init_node('controller_test')
    loop()

def loop():

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        t = rospy.Time.now()
        val = 5.0 * math.sin(2*math.pi*t.to_sec()) + 10

        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = val

        cmd_pub.publish(vel_cmd)

        rate.sleep


if __name__ == '__main__':
    cmd_pub  = rospy.Publisher('/twist_cmd', TwistStamped, queue_size=10)

    try:
        run()
    except rospy.ROSInterruptException:
        pass
