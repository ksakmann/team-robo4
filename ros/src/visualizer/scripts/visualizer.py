#!/usr/bin/env python 

import sys

import rospy
from qtplot import App
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg

# import time
# import numpy as np


class Visualizer(object):
    def __init__(self, argv):
        rospy.init_node('visualizer')

        rate = rospy.Rate(10) # 50Hz

        app = QtGui.QApplication(sys.argv)
        self.thisapp = App()
        self.thisapp.show()

        # rospy.spin()
        while not rospy.is_shutdown():
            self.thisapp._update()
            pg.QtGui.QApplication.processEvents()
            rate.sleep()


if __name__ == '__main__':
    try:
        print(sys.argv)
        Visualizer(sys.argv)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visualizer node.')
