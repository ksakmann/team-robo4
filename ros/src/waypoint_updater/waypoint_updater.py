#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
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


def qe(a, b, c):
    tmp = b*b - 4*a*c
    if tmp < 0:
        rospy.logerr('Attempting square root of -ve number %s', tmp)

    return (math.sqrt(min(0,tmp)) - b) / (2*a)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose'  , PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane       , self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint' , Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.tw_id = 0
        self.ow_id = 0
        self.accel_limit = 4.0 # m/sec^2
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
        self.target_velocity = 10.0 * 0.4407	# target velocity is 10m/sec

        rospy.spin()

    def pose_cb(self, msg):

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        # Note on quaternions: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        self.orientation = msg.pose.orientation
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(quaternion)

        rospy.logdebug('pose : %s, %s, %s', self.x, self.y, self.z)

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

    def get_local_coordinates(self,wp):
        # Two trafos  like in MPC project
        # 1. shift
        x,y = wp.pose.pose.position.x,wp.pose.pose.position.y
        xp,yp = x-self.x,y-self.y

        # 2. rotation
        # coordinates transform inversely to basis vectors when the rotation is +yaw
        xpp = math.cos(self.yaw) * xp + math.sin(self.yaw) * yp
        ypp = -math.sin(self.yaw) * xp + math.cos(self.yaw) * yp

        return xpp, ypp

    def get_closest_waypoint_index(self):
        rospy.loginfo('x,y : %s, %s ', self.x, self.y)

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

        rospy.loginfo('closest : %s, %s ',
                      self.waypoints[self.closest].pose.pose.position.x,
                      self.waypoints[self.closest].pose.pose.position.y)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.no_waypoints = len(msg.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.final_waypoints = self.get_next_waypoints()
        self.tw_id = msg.data

        if self.waypoints is not None:
            v0 = self.get_waypoint_velocity(self.waypoints[self.closest]) * 0.4407 # m/s
        else:
            v0 = self.target_velocity

        rospy.loginfo('traffic_waypoint: %s', self.tw_id)
        rospy.loginfo('closest index: %s', self.closest)
        rospy.loginfo('initial v: %s', v0)

        if self.final_waypoints is not None:
            if self.tw_id != -1:
                # compute distance from current position to red tl.
                d = self.distance(self.waypoints, self.closest, self.tw_id)
                rospy.loginfo('tl distance: %s', d)

                if self.chk_stp(v0, d):  #check whether or not can stop.
                    # set deceleration value.
                    if d != 0:
                        decel = v0 * v0 /2/d
                    else:
                        decel = 0
                    rospy.loginfo('deceleration: %s', decel)

                    for i in range(len(self.final_waypoints.waypoints)):
                        # distance from closest id to the point where set the vehicle speed this time.
                        if (self.tw_id - self.closest != 0):
                            d_int = d / (self.tw_id - self.closest) * i
                        else:
                            d_int = 0

                        # Solve the quadratic equation to find the time required to travel
                        # a certain distance.
                        #if d > 80:	# Don't decelerate when tl is more than 80m away.
                        #    set_v = self.target_velocity
                        if ((decel != 0) & (d_int <= d)):
                            delta_t = qe(-decel/2, v0, -d_int)
                            set_v = max(v0 - decel * delta_t, 0)
                            self.set_waypoint_velocity(self.final_waypoints.waypoints, i, set_v)
                        else:
                            set_v = 0
                        #rospy.logwarn('index: %s, set v: %s', i, set_v)
                        self.set_waypoint_velocity(self.final_waypoints.waypoints, i, set_v)
            else:
                # Accelerate to set speed.
                for i in range(len(self.final_waypoints.waypoints)):
                    #d_int = self.distance(self.waypoints, self.closest, self.closest + i)
                    #delta_t = qe(self.accel_limit/2, v0, -d_int)
                    #set_v = min(self.target_velocity, v0 + self.accel_limit * delta_t)
                    # accelerate with DBW_node.
                    set_v = self.target_velocity
                    self.set_waypoint_velocity(self.final_waypoints.waypoints, i, set_v)

            # rospy.logwarn('set v : %s', self.get_waypoint_velocity(self.final_waypoints.waypoints[0]))
            #if ((self.tw_id != -1) & (self.tw_id > self.closest)):
            #    rospy.logwarn('set tl v : %s', self.get_waypoint_velocity(self.final_waypoints.waypoints[self.tw_id - self.closest]))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.ow_id = msg.data
        # TODO: Implement code to stop before the obstacle. Almost the same as traffic_cb?!
        if self.waypoints is not None:
            v0 = self.get_waypoint_velocity(self.waypoints[self.closest]) * 0.44704 # mph to m/s
        else:
            v0 = self.target_velocity

        rospy.loginfo('obstacle waypoint: %s', self.ow_id)
        #rospy.logwarn('closest index: %s', self.closest)
        #rospy.logwarn('initial v: %s', v0)

        if self.final_waypoints is not None:
            if self.ow_id != -1:
                # compute distance from current position to red tl.
                d = self.distance(self.waypoints, self.closest, self.ow_id)
                rospy.loginfo('obstacle distance: %s', d)

                # set deceleration value.
                if d != 0:
                    decel = v0 * v0 /2/d
                else:
                    decel = 0
                rospy.loginfo('deceleration: %s', decel)

                for i in range(len(self.final_waypoints.waypoints)):
                    # distance from closest id to the point where set the vehicle speed this time.
                    if (self.ow_id - self.closest != 0):
                        d_int = d / (self.ow_id - self.closest) * i
                    else:
                        d_int = 0

                    # Solve the quadratic equation to find the time required to travel
                    # a certain distance.
                    if ((decel != 0) & (d_int <= d)):
                        delta_t = qe(-decel/2, v0, -d_int)
                        set_v = max(v0 - decel * delta_t, 0)
                        self.set_waypoint_velocity(self.final_waypoints.waypoints, i, set_v)
                    else:
                        set_v = 0
                    #rospy.logwarn('index: %s, set v: %s', i, set_v)
                    self.set_waypoint_velocity(self.final_waypoints.waypoints, i, set_v)
            else:
                # Accelerate to set speed.
                for i in range(len(self.final_waypoints.waypoints)):
                    # accelerate with DBW_node.
                    set_v = self.target_velocity
                    self.set_waypoint_velocity(self.final_waypoints.waypoints, i, set_v)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

        
    def distance(self, waypoints, wp1, wp2):
        # TODO: Returns distance between waypoints indexed wp1 and wp2
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def chk_stp(self, v_init, dist):
        # Determine whether or not we can stop under the limit deceleration.
        stp_t = v_init / self.accel_limit
        d_stp = v_init * stp_t - self.accel_limit * stp_t * stp_t / 2

        if d_stp>dist:
            return False
        else:
            return True


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
