#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

import math, tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.WARN)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.tw_id = 0
        self.ow_id = 0
        
        self.accel_limit = 4.0 # m/sec^2

        self.base_waypoints = Lane()
        self.final_waypoints = Lane()

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
	self.target_velocity = 10.0 * 0.44704	# target velocity is 10mph. in our unit, m/sec

        # standard loop to publish data from ros pub/sub tutorial
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            lane = self.get_next_waypoints()
            if lane is not None:
                self.final_waypoints_pub.publish(lane)

            rate.sleep()

    def pose_cb(self, msg):

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        # Note on quaternions: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        self.orientation = msg.pose.orientation
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(quaternion)
        rospy.logwarn('pose : %s, %s, %s', self.x, self.y, self.z)

    def get_next_waypoints(self):

        self.get_closest_waypoint_index()
        ind = self.closest

        if self.yaw is None or self.waypoints is None or ind is None:
            rospy.logwarn('yaw,closest : %s, %s', self.yaw, self.closest)
            return

        wp = self.waypoints[ind]
        xpp,_ = self.get_local_coordinates(wp)

        if xpp < 0:
            ind = (ind + 1) % self.no_waypoints

        lane = Lane()
        for i in range(LOOKAHEAD_WPS):
            lane.waypoints.append(self.waypoints[(ind + i) % self.no_waypoints])

        rospy.logwarn('wp.pose.pose.position.x,wp.pose.pose.position.y : %s, %s', wp.pose.pose.position.x, wp.pose.pose.position.y)
        return lane



    def get_local_coordinates(self,wp):
        # Two trafos  like in MPC project
        # 1. shift
        x,y = wp.pose.pose.position.x,wp.pose.pose.position.y
        xp,yp = x-self.x,y-self.y

        # 2. rotation
        # coordinates transform inversely to basis vectors when the rotation is +yaw
        xpp = math.cos(self.yaw) * xp + math.sin(self.yaw) * yp
        ypp = -math.sin(self.yaw) * xp + math.cos(self.yaw) * yp

        return xpp,ypp


    def get_closest_waypoint_index(self):

        rospy.logwarn('x,y : %s, %s ', self.x, self.y)

        if self.x is None or self.y is None or self.waypoints is None:
            return

        distance = 1E10
        self.closest = -1000
        for ind, wp in enumerate(self.waypoints):
            wp_distance = math.sqrt((self.x - wp.pose.pose.position.x) ** 2 \
                                    + (self.y - wp.pose.pose.position.y) ** 2 \
                                    + (self.z - wp.pose.pose.position.z) ** 2)
            if wp_distance < distance:
                distance = wp_distance
                self.closest = ind
        rospy.logwarn('closest : %s, %s ', self.waypoints[self.closest].pose.pose.position.x,
                      self.waypoints[self.closest].pose.pose.position.y)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.no_waypoints = len(msg.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.tw_id = msg
        v0 = get_waypoint_velocity(self.base_waypoints.waypoints[self.closest])

        if self.tw_id != -1:
            # compute distance from current position to red tl.
            d = self.distance(self.base_waypoints, self.closest, self.tw_id)

            # solve the quadratic equation.
            qe = lambda a,b c: (math.srqt(b*b + 4*a*c) - b) /2/a

            if chk_stp(v0, d):  #check whether or not can stop.
                for i, wp in enumerate(self.final_waypoints.waypoints):
                    # distance from near_id to the point where set the vehicle speed this time.
                    d_int = d / (self.tw_id - self.closest - i)
                    decel = v0 * v0 /2/d
                    # Solve the quadratic equation to find the time required to travel
                    # a certain distance.
                    delta_t = qe(decel/2, v0, d_int)
                    set_v = max(v0 - decel * delta_t, 0)
                    set_waypoint_velocity(wp, i, set_v)

        else:
            # Accelerate to set speed.
            for i, wp in enumerate(self.final_waypoints.waypoints):

                d = self.distance(self.base_waypoints, self.closest, self.closest + i)
                delta_t = qe(self.accel_limit, v0, d_int)
                set_v = min(self.target_velocity, v0 + self.accel_limit * delta_t)
                set_waypoint_velocity(wp, i, set_v)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.ow_id = msg
	# TODO: Implement code to stop before the obstacle. Almost the same as traffic_cb?!

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

        # TODO: is there any use for this? Returns distance between waypoints indexed wp1 and wp2
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
