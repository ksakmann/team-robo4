#!/usr/bin/env python 

import sys
import numpy as np

import rospy
from styx_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped, Pose
from qtplot import App
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg


# import time
# import numpy as np


class Visualizer(object):
    def __init__(self, argv):
        rospy.init_node('visualizer')

        app = QtGui.QApplication(sys.argv)
        self.thisapp = App()
        self.thisapp.show()

        # self.waypoints = None
        self.waypoints_sub = rospy.Subscriber('/base_waypoints', Lane       , self.waypoints_cb)
        self.pose_sub      = rospy.Subscriber('/current_pose'  , PoseStamped, self.pose_cb)

        rate = rospy.Rate(10) # 50Hz

        # rospy.spin()
        while not rospy.is_shutdown():
            self.thisapp._update()
            pg.QtGui.QApplication.processEvents()
            rate.sleep()


    def waypoints_cb(self, msg):
        # self.waypoints = msg
        rospy.loginfo('Waypoints message received')
        x = np.asarray([this.pose.pose.position.x for this in msg.waypoints])
        y = np.asarray([this.pose.pose.position.y for this in msg.waypoints])
        self.thisapp.plotMap(x, y)


    def pose_cb(self, msg):
        # rospy.loginfo('Vehicle pose received')
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.thisapp.plotVehicle(x, y)


if __name__ == '__main__':
    try:
        print(sys.argv)
        Visualizer(sys.argv)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visualizer node.')
