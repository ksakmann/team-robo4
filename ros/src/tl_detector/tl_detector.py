#!/usr/bin/env python

import tf
import cv2
import yaml
import math
import numpy as np
import os

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from light_classification.tl_classifier import TLClassifier


STATE_COUNT_THRESHOLD = 3


def distance_xy(a, b):
    """Return the euclidean distance between two geometry_msgs.Pose objects"""
    return np.sqrt((a.position.x - b.position.x)**2 + (a.position.y - b.position.y)**2)


def distance_xyz(a, b):
    """Return the euclidean distance between two geometry_msgs.Pose objects"""
    return np.sqrt((a.position.x - b.position.x)**2 + (a.position.y - b.position.y)**2 + + (a.position.z - b.position.z)**2)


def distance_ahead(ref_wp, wp):
    """Return the distance of wp relative to ref_wp in the ref_wp reference frame."""

    dx = wp.position.x - ref_wp.position.x
    dy = wp.position.y - ref_wp.position.y
    quaternion = [ref_wp.orientation.x, ref_wp.orientation.y, ref_wp.orientation.z, ref_wp.orientation.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

    dx_ref =  dx*math.cos(yaw) + dy*math.sin(yaw)
    dy_ref = -dx*math.sin(yaw) + dy*math.cos(yaw)

    return dx_ref, dy_ref


def pose_list(xy_list):
        """Returns a list of Pose objects given an array of x, y values
        
        Args:
            xy_list (list):

        """
        
        list = []
        for xy in xy_list:
            lightstop_pose = Pose()
            lightstop_pose.position.x = xy[0]
            lightstop_pose.position.y = xy[1]
            list.append(lightstop_pose)

        return list


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.busy = False

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.lightstops_pose = pose_list(self.config['stop_line_positions'])
        self.lightstops_wp_index = []
        self.is_lightstops_indexed = False

        self.image_pub = rospy.Publisher("image_classified_new", Image)
        self.pose_sub    = rospy.Subscriber('/current_pose'          , PoseStamped      , self.pose_cb)
        self.wp_sub      = rospy.Subscriber('/base_waypoints'        , Lane             , self.waypoints_cb)
        self.traffic_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.img_sub     = rospy.Subscriber('/image_color'           , Image            , self.image_cb)        

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint'  , Int32       , queue_size=1)
        self.upcoming_light_pub     = rospy.Publisher('/traffic_waypoint_2', TrafficLight, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.save_images = rospy.get_param("~save_image", False)
        self.saved_image_counter = 1
        self.saved_image_limit = 500

        # Set to true to use grount truth light states otherwise use classifier
        self.use_ground_truth = rospy.get_param("~use_ground_truth")

        self.previous_light_state = self.state
        self.previous_closest_lightstop_wp_index = self.last_wp

        self.light_classifier = None
        self.light_classifier = TLClassifier()

        rospy.spin()

    def lightstops_index(self):
        """Indexes the lightstops_wp_index property"""

        for lightstop_pose in self.lightstops_pose:
            min_dist = 1e10
            for i, wp in enumerate(self.waypoints.waypoints):
                dist = distance_xy(lightstop_pose, wp.pose.pose)
                if dist < min_dist:
                    min_dist = dist
                    index = i
            self.lightstops_wp_index.append(index)
        self.is_lightstops_indexed = True

    def pose_cb(self, msg):
        """Vehicle pose callback."""

        self.pose = msg
        # self.pose_idx = self.get_closest_waypoint_birectional(msg.pose)

    def waypoints_cb(self, waypoints):
        """Base waypoints call back."""
        self.waypoints = waypoints
        if not self.is_lightstops_indexed:
            self.lightstops_index()

    def traffic_cb(self, msg):
        """Traffic lights status callback.

        The /vehicle/traffic_lights topic provides you with the location of
        the traffic light in 3D map space and helps you acquire an accurate 
        ground truth data source for the traffic light classifier by sending
        the current color state of all traffic lights in the simulator. When
        testing on the vehicle, the color state will not be available. You'll
        need torely on the position of the light and the camera image to
        predict it.
        
        """
        
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image.
        
        The index of the waypoint closest to the red light's stop line gets
        published to the /traffic_waypoint topic.

        Args:
            msg (Image): image from car-mounted camera

        """

        self.has_image = True
        self.camera_image = msg

        state, closest_lightstop_wp_index = self.process_traffic_lights()
        # state = out[0]
        # light_wp_index = out[1]
        # light_state_gt = out[2]
        # closest_lightstop_wp_index_gt = out[3]
        # light_pose = out[4]
        
        # DEVELOPMENT - remove these lines of code to use the classifier instead of ground truth
        # state = light_state_gt
        # light_wp_index = closest_lightstop_wp_index_gt

        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur STATE_COUNT_THRESHOLD number
        # of times till we start using it. Otherwise the previous stable state is
        # used

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            closest_red_lightstop_wp_index = closest_lightstop_wp_index if state == TrafficLight.RED else -1
            self.last_wp = closest_red_lightstop_wp_index
            self.upcoming_red_light_pub.publish(Int32(closest_red_lightstop_wp_index))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1

        traffic_light = TrafficLight()
        traffic_light.header.stamp = rospy.Time.now()
        traffic_light.header.frame_id = '/world'
        traffic_light.state = TrafficLight.UNKNOWN

        if closest_lightstop_wp_index != -1:
            # For visualization publish light location and color
            # Only publish traffic light for visualization if light is red, 
            # green or yellow, and if we found a nearby lightstop waypoint
            traffic_light.state = state
            light_pose = self.waypoints.waypoints[closest_lightstop_wp_index]
            traffic_light.pose.pose = light_pose.pose.pose

        self.upcoming_light_pub.publish(traffic_light)

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location.

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        rot = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        x = 0
        y = 0
        if (trans != None):
            rot_eu = tf.transformations.euler_from_quaternion(rot)
            s_yaw = math.sin(rot_eu[2])
            c_yaw = math.cos(rot_eu[2])

            piw_x = point_in_world.x
            piw_y = point_in_world.y
            piw_z = point_in_world.z

            t_x = trans[0]
            t_y = trans[1]
            t_z = trans[2]

            rot_trans = (piw_x*c_yaw - piw_y*s_yaw + t_x,
                         piw_x*s_yaw + piw_y*c_yaw + t_y,
                         piw_z + t_z)

            # rospy.logwarn('fx=%f, fy=%f', fx, fy)
            x = int(fx * -rot_trans[1]/rot_trans[0] + image_width/2)
            y = int(fy * -rot_trans[2]/rot_trans[0] + image_height/2)
            # rospy.logwarn('x=%f, y=%f', x, y)

        return (x, y)

    def get_light_state_ground_truth(self):
        """Returns the state of the traffic light"""

        closest_lightstop_wp_index = -1
        closest_light_state = TrafficLight.UNKNOWN

        if self.pose is None or not self.is_lightstops_indexed:
            return closest_light_state, closest_lightstop_wp_index

        car_pose = self.pose.pose

        # Find traffic lights within this distance in the vehicle ref frame
        min_dx = 100 
        min_dy = 20  

        closest_light_pose = None
        for light in self.lights:
            light_pose = light.pose.pose
            dx, dy = distance_ahead(car_pose, light_pose)

            # If the light is ahead of vehicle, less than previously know minimum x dist
            # and less than a lateral distance threshold (don't detect lights that are 
            # very far away laterally)
            if dx < min_dx and dy < min_dy and dx > 0:
                min_dx = dx
                closest_light_pose = light_pose
                closest_light_state = light.state
        
        if closest_light_pose: # If a light was found let's find the closest stop
            # rospy.loginfo('Traffic light found %d m away', min_dist)
            min_dist = 50 # Find closest lightstop within this distance
            for i, lightstop_pose in enumerate(self.lightstops_pose):
                dist = distance_xy(lightstop_pose, closest_light_pose)
                dx, dy = distance_ahead(car_pose, lightstop_pose)
                # If this lightstop is the closest one so far to the light stop
                # and it's ahead of the car...
                if dist < min_dist and dx > 0:
                    min_dist = dist
                    # closest_lightstop_pose = lightstop_pose
                    closest_lightstop_wp_index = self.lightstops_wp_index[i]

            # if closest_lightstop_pose is None:
                # pass
                # rospy.logwarn('No stopping line was found for visible traffic light')
            # else:
                # dist_veh_to_light_stop = distance_xy(closest_lightstop_pose, car_pose)
                # rospy.loginfo('Traffic light stop found %d m away', dist_veh_to_light_stop)
        else:
            # rospy.loginfo('No traffic light found')
            pass
        
        return closest_light_state, closest_lightstop_wp_index

    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if not self.has_image:
            self.prev_light_loc = None
            return False
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        except CvBridgeError as e:
            print(e)

        # rospy.logwarn(len(cv_image))
        # output image will be of size 800x400

        # save 100 images for training purposes
        # print('Save image? %r', self.save_images)
        if self.save_images == True and self.saved_image_counter <= self.saved_image_limit:
            rospy.loginfo('saving images')
            if not (os.path.exists("./tl_images")):
                os.mkdir("./tl_images")
            cv2.imwrite("./tl_images/image{0:0>4}.jpeg".format(self.saved_image_counter), cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR))
            self.saved_image_counter += 1

        # x, y = self.project_to_image_plane(light.pose.pose.position)
        # rospy.loginfo('project to image plane x, y = %f, %f', x, y)

        #Get classification
        if self.light_classifier is not None:
            classified_light_state = self.light_classifier.get_classification(cv_image)
        else:
            rospy.loginfo('light_classifier is None')
            classified_light_state = TrafficLight.UNKNOWN
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e)

        # simulator_light_state = self.get_light_state_from_simulator(light)

        # if classified_light_state != simulator_light_state:
        #     rospy.logwarn('Classified light state: %d, ground truth light state: %d', classified_light_state, simulator_light_state)

        # rospy.loginfo('Classified light state: %d, ground truth light state: %d', classified_light_state, simulator_light_state)

        closest_lightstop_wp_index = -1

        if classified_light_state != 4:
            car_pose = self.pose.pose
            min_dx = 100
            min_dy = 20
            for i, lightstop_pose in enumerate(self.lightstops_pose):
                dx, dy = distance_ahead(car_pose, lightstop_pose)
                # If the lightstop is ahead of vehicle, less than previously know minimum x dist
                # and less than a lateral distance threshold (don't detect lights that are 
                # very far away laterally)
                if dx < min_dx and dy < min_dy and dx > 0:
                    min_dx = dx
                    closest_lightstop_pose = lightstop_pose
                    closest_lightstop_wp_index = self.lightstops_wp_index[i]
            
        return classified_light_state, closest_lightstop_wp_index

    def process_traffic_lights(self):

        if self.use_ground_truth:
            light_state, closest_lightstop_wp_index = self.get_light_state_ground_truth()
        else:
            if not self.busy:
                self.busy = True
                light_state, closest_lightstop_wp_index = self.get_light_state()
                self.previous_light_state = light_state
                self.previous_closest_lightstop_wp_index = closest_lightstop_wp_index
                self.busy = False
            else:
                light_state, closest_lightstop_wp_index = self.previous_light_state,self.previous_closest_lightstop_wp_index
        
        return light_state, closest_lightstop_wp_index


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
