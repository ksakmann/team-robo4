#!/usr/bin/env python 

import sys
import numpy as np

import rospy
from styx_msgs.msg import Lane, TrafficLight
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
        self.waypoints_sub = rospy.Subscriber('/base_waypoints'    , Lane        , self.waypoints_cb)
        self.pose_sub      = rospy.Subscriber('/current_pose'      , PoseStamped , self.pose_cb)
        self.traffic_sub   = rospy.Subscriber('/traffic_waypoint_2', TrafficLight, self.traffic_cb)

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            pg.QtGui.QApplication.processEvents()
            rate.sleep()


    def waypoints_cb(self, msg):
        x = np.asarray([this.pose.pose.position.x for this in msg.waypoints])
        y = np.asarray([this.pose.pose.position.y for this in msg.waypoints])
        self.thisapp.plotMap(x, y)


    def pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.thisapp.plotVehicle(x, y)

    
    def traffic_cb(self, msg):
        b_clear = False
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if msg.state == 0:
            color = 'r'
        elif msg.state == 1:
            color = 'y'
        elif msg.state == 2:
            color = 'g'
        elif msg.state == 4:
            b_clear = True
        else:
            rospy.error('Unexpected traffic light status')

        if not b_clear:
            self.thisapp.plotTrafficLight(x, y, color)
        else:
            self.thisapp.plotTrafficLightClear()


if __name__ == '__main__':
    try:
        Visualizer(sys.argv)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visualizer node.')
