#!/usr/bin/env python

import math, tf

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO Should we be using x, y, z for distance?
TODO Implement distance_ahead instead of distance?
'''


LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
NOMINAL_DECCEL = 0.25 
TARGET_VELOCITY = 20.0 * 0.4407
STOPPING_DISTANCE_BUFFER = 2 


def distance(waypoints, wp1, wp2):
        # TODO: Returns distance between waypoints indexed wp1 and wp2
        # Calculate distance along path
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        # dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i

        return dist


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose'  , PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane       , self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32       , self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_vel_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.tw_id = 0
        self.ow_id = 0
        self.deccel = NOMINAL_DECCEL
        self.stopping_distance_buffer = STOPPING_DISTANCE_BUFFER
        self.final_waypoints = Lane()
        self.current_velocity = 0
        self.target_velocity = TARGET_VELOCITY

        self.x = None
        self.y = None
        self.z = None  # z seems to be always zero for all base waypoints
        self.orientation = None
        self.roll = None
        self.pitch = None
        self.yaw = None  # the orientation in the plane z = 0 (phi)
        self.waypoints = None
        self.no_waypoints = None
        self.closest = None	# closest waypoint index to current position.

        rospy.spin()

    def pose_cb(self, msg):

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        # Note on quaternions: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        self.orientation = msg.pose.orientation
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(quaternion)

        # rospy.logdebug('pose : %s, %s, %s', self.x, self.y, self.z)

        # if self.tw_id == -1:
        #     self.final_waypoints = self.get_next_waypoints()

        if self.final_waypoints is not None:
            self.final_waypoints_pub.publish(self.final_waypoints)

    def get_next_waypoints(self):

        self.get_closest_waypoint_index()
        ind = self.closest

        if self.yaw is None or self.waypoints is None or ind is None:
            # rospy.logwarn('yaw,closest : %s, %s', self.yaw, self.closest)
            return

        wp = self.waypoints[ind]
        xpp,_ = self.get_local_coordinates(wp)

        if xpp < 0:
            ind = (ind + 1) % self.no_waypoints

        lane = Lane()
        for i in range(LOOKAHEAD_WPS):
            lane.waypoints.append(self.waypoints[(ind + i) % self.no_waypoints])

        # rospy.logwarn('wp.pose.pose.position.x,wp.pose.pose.position.y : %s, %s', wp.pose.pose.position.x, wp.pose.pose.position.y)
        return lane

    def get_next_waypoint_indices(self):

        self.get_closest_waypoint_index()
        ind = self.closest

        if self.yaw is None or self.waypoints is None or ind is None:
            # rospy.logwarn('yaw,closest : %s, %s', self.yaw, self.closest)
            return

        wp = self.waypoints[ind]
        xpp,_ = self.get_local_coordinates(wp)

        if xpp < 0:
            ind = (ind + 1) % self.no_waypoints

        indices = [i % self.no_waypoints for i in range(ind, ind + LOOKAHEAD_WPS)]  

        return indices

    def get_local_coordinates(self, wp):
        # Two trafos  like in MPC project
        # 1. shift
        x, y = wp.pose.pose.position.x, wp.pose.pose.position.y
        xp, yp = x - self.x, y - self.y

        # 2. rotation
        # coordinates transform inversely to basis vectors when the rotation is +yaw
        xpp = math.cos(self.yaw) * xp + math.sin(self.yaw) * yp
        ypp = -math.sin(self.yaw) * xp + math.cos(self.yaw) * yp

        return xpp, ypp

    def get_closest_waypoint_index(self):
        # rospy.loginfo('x,y : %s, %s ', self.x, self.y)

        if self.x is None or self.y is None or self.waypoints is None:
            return

        distance = 1E10
        self.closest = -1000
        for ind, wp in enumerate(self.waypoints):
            wp_distance = math.sqrt((self.x - wp.pose.pose.position.x) ** 2 + \
                                    (self.y - wp.pose.pose.position.y) ** 2 + \
                                    (self.z - wp.pose.pose.position.z) ** 2)
            if wp_distance < distance:
                distance = wp_distance
                self.closest = ind

        # rospy.loginfo('closest : %s, %s ',
        #               self.waypoints[self.closest].pose.pose.position.x,
        #               self.waypoints[self.closest].pose.pose.position.y)

    def waypoints_cb(self, msg):
        rospy.loginfo("Basewaypoints set")
        self.waypoints = msg.waypoints
        self.no_waypoints = len(msg.waypoints)

    def traffic_cb(self, msg):
        self.final_waypoints = self.get_next_waypoints()
        next_indices = self.get_next_waypoint_indices()
        self.tw_id = msg.data

        is_red_light_present = self.tw_id != -1

        if self.final_waypoints is not None:
            if is_red_light_present:
                for i, wp_index in enumerate(next_indices):
                    # Calculate distance to red light and factor in a safety margin buffer
                    d = distance(self.waypoints, wp_index, self.tw_id) - self.stopping_distance_buffer
                    if d > 0:
                        # velocity we could be traveling and still be able to stop at the desired rate
                        target_velocity_deccel = math.sqrt(d*2*self.deccel) 
                    else:
                        # traffic light is behind us
                        target_velocity_deccel = 0.0
                    # Finally, take the minimum from the accel profile and the cruising speed profile
                    target_velocity = min([self.target_velocity, target_velocity_deccel])
                    self.set_waypoint_velocity(self.final_waypoints.waypoints, i, target_velocity)
            else:
                for i, wp_index in enumerate(next_indices):
                    # Finally, take the minimum from the accel profile and the cruising speed profile
                    self.set_waypoint_velocity(self.final_waypoints.waypoints, i, self.target_velocity)

    def current_vel_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
