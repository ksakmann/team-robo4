#!/usr/bin/env python 

import sys
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from qtplot import App
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg

from styx_msgs.msg import Lane, TrafficLight


class Visualizer:
    def __init__(self, argv):
        rospy.init_node('visualizer')

        self.argv = argv

        # self.waypoints = None
        self.waypoints_sub = rospy.Subscriber('/base_waypoints'    , Lane        , self.waypoints_cb)
        self.pose_sub      = rospy.Subscriber('/current_pose'      , PoseStamped , self.pose_cb)
        self.traffic_sub   = rospy.Subscriber('/traffic_waypoint_2', TrafficLight, self.traffic_cb)

        self.state = TrafficLight.UNKNOWN

        self.rate = rospy.Rate(1)

    def run_loop(self):

        app = QtGui.QApplication(self.argv)
        self.thisapp = App()
        self.thisapp.show()
        QtGui.QApplication.instance().processEvents()
        while not rospy.is_shutdown():
            # pg.QtGui.QApplication.processEvents()
            QtGui.QApplication.instance().processEvents()
            self.rate.sleep()

    def waypoints_cb(self, msg):
        x = np.asarray([this.pose.pose.position.x for this in msg.waypoints])
        y = np.asarray([this.pose.pose.position.y for this in msg.waypoints])
        self.thisapp.plotMap(x, y)

    def pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.thisapp.plotVehicle(x, y)
    
    def traffic_cb(self, msg):

        if self.state != msg.state: # state change, game on
            
            self.state = msg.state

            b_clear = False
            
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
                rospy.loginfo('Plot traffic light visual')
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                self.thisapp.plotTrafficLight(x, y, color)
            else:
                rospy.loginfo('Clearing traffic light visual')
                self.thisapp.plotTrafficLightClear()

        else: 

            pass


if __name__ == '__main__':
    try:
        global graph_obj
        graph_obj = Visualizer(sys.argv)
        graph_obj.run_loop()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visualizer node.')
