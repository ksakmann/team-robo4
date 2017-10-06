#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

import tf
from geometry_msgs.msg import Pose, Quaternion
from styx_msgs.msg import Lane, Waypoint

CSV_HEADER = ['x', 'y', 'z', 'yaw']
MAX_DECEL = 1.0


class main(object):

    def __init__(self, file):
        self.velocity = 30
        self.waypoints = self.load_waypoints(file)
        
        x1 = 1867.27
        y1 = 1144.95

        pose = Pose()
        pose.position.x = x1
        pose.position.y = y1
        idx = self.get_closest_waypoint(pose)

        wp = self.waypoints[idx]
        x2 = wp.pose.pose.position.x
        y2 = wp.pose.pose.position.y

        self.plot(x1, y1, x2, y2)


    def plot(self, x1, y1, x2, y2):
        
        print('x1 = %f, y1 = %f',x1, y1)
        print('x2 = %f, y2 = %f',x2, y2)

        x = [entry.pose.pose.position.x for entry in self.waypoints]
        y = [entry.pose.pose.position.y for entry in self.waypoints]

        plt.plot(x, y, 'o', markerfacecolor='None')
        plt.plot(x1, y1, 's', x2, y2, '*')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0., 0., yaw)


    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def load_waypoints(self, fname):
        waypoints = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                p.pose.pose.position.z = float(wp['z'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x = float(self.velocity*0.27778)

                waypoints.append(p)
        return self.decelerate(waypoints)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist) * 3.6
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement - Done
        # Iterate the base_waypoints' x value with current position's x value and find the closest
        # match, and pick that waypoint location index.  
        min_idx = 0
        min_dist = None
        cur_x = pose.position.x
        cur_y = pose.position.y
        if self.waypoints is not None:
            for i, wp in enumerate(self.waypoints):
                wp_x = wp.pose.pose.position.x
                wp_y = wp.pose.pose.position.y
                dist = np.sqrt((cur_x - wp_x)**2 + (cur_y - wp_y)**2)
                if min_dist is None or min_dist >= dist:
                    min_dist = dist
                    min_idx = i
        
        # check whether the identified index is behind the current position, if so, move it by 1 index
        # https://gamedev.stackexchange.com/questions/75072/how-can-i-compare-two-quaternions-for-logical-equality
        # rospy.logwarn('min_idx before = %d', min_idx)
        eps = 1e-12
        if self.waypoints is not None:
            q1 = self.waypoints[min_idx].pose.pose.orientation
            q2 = pose.orientation
            q1_a = np.array([q1.x, q1.y, q1.z, q1.w])
            q2_a = np.array([q2.x, q2.y, q2.z, q2.w])
            direction = abs(np.dot(q1_a, q2_a))
            #rospy.logwarn('calculated direction %f', direction)
            wp_x = self.waypoints[min_idx].pose.pose.position.x
            if direction > 1-eps:
                if wp_x < cur_x:
                    min_idx += 1
                else:
                    min_idx -= 1
            else:
                if wp_x < cur_x:
                    min_idx -= 1
                else:
                    min_idx += 1

        # rospy.logwarn('min_idx after = %d', min_idx)
        return min_idx



if __name__ == "__main__":
    import sys
    import os
    file = os.path.join(os.getcwd(), sys.argv[1])
    main(file)