#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
import os

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose_idx = None
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.save_images = False
        self.saved_image_counter = 1
        self.saved_image_limit = 100

        self.light_classifier = TLClassifier()

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        # rospy.logwarn('curpos x=%f, y=%f, a_x=%f, a_y=%f, a_z=%f', msg.pose.position.x, msg.pose.position.y, \
        #     msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        self.pose_idx = self.get_closest_waypoint_birectional(msg.pose)
        # rospy.logwarn('cur pos idx=%d', self.pose_idx)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # rospy.logwarn('===============')
        # rospy.logwarn(self.lights)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

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
            for i, wp in enumerate(self.waypoints.waypoints):
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
            q1 = self.waypoints.waypoints[min_idx].pose.pose.orientation
            q2 = pose.orientation
            q1_a = np.array([q1.x, q1.y, q1.z, q1.w])
            q2_a = np.array([q2.x, q2.y, q2.z, q2.w])
            direction = abs(np.dot(q1_a, q2_a))
            #rospy.logwarn('calculated direction %f', direction)
            wp_x = self.waypoints.waypoints[min_idx].pose.pose.position.x
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

    def get_closest_waypoint_birectional(self, pose):
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
            for i, wp in enumerate(self.waypoints.waypoints):
                wp_x = wp.pose.pose.position.x
                wp_y = wp.pose.pose.position.y
                dist = np.sqrt((cur_x - wp_x)**2 + (cur_y - wp_y)**2)
                if min_dist is None or min_dist >= dist:
                    min_dist = dist
                    min_idx = i
        
        # rospy.logwarn('min_idx after = %d', min_idx)
        return min_idx

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

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

    def get_light_state_from_simulator(self, light):
        # assuming that simulator publishes the traffic light status in the sequence of nearest available light.
        # rospy.logwarn('ligt data type is --------%s\\n', light)
        tl_state = -1
        # cnt = 1
        for tl in self.lights:
            distance = np.sqrt((tl.pose.pose.position.x - light.pose.pose.position.x)**2 + (tl.pose.pose.position.y - light.pose.pose.position.y)**2)
            # rospy.logwarn('distance=%d, tl.state=%d', distance, tl.state)
            if (distance < 50): 
                tl_state = tl.state
                break
            # cnt +=1
            
        # rospy.logwarn('traffic light state = %d', tl_state)
        # rospy.logwarn(tl_state)
        return tl_state


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #TODO use light location to zoom in on traffic light in image
        # rospy.logwarn(len(cv_image))
        cv_image = cv_image[201:600, :, :]
        # output image will be of size 800x400

        # save 100 images for training purposes
        print('Save image? %r', self.save_images)
        if self.save_images == True and self.saved_image_counter <= self.saved_image_limit:
            rospy.loginfo('saving images')
            if not (os.path.exists("./tl_images")):
                os.mkdir("./tl_images")
            cv2.imwrite("./tl_images/image{}.jpg".format(self.saved_image_counter), cv_image)
            self.saved_image_counter += 1

        x, y = self.project_to_image_plane(light.pose.pose.position)
        # rospy.loginfo('project to image plane x, y = %f, %f', x, y)

        #Get classification
        classified_light_state = self.light_classifier.get_classification(cv_image)
        simulator_light_state = self.get_light_state_from_simulator(light)

        if classified_light_state != simulator_light_state:
            rospy.logwarn('Classified light state: %d, ground truth light state: %d', classified_light_state, simulator_light_state)

        return simulator_light_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        cls_light_wpx = None
        cls_light_stop_wpx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

            #TODO find the closest visible traffic light (if one exists)
            for lp in self.lights:
                light_wpx = self.get_closest_waypoint_birectional(lp.pose.pose)
                if light_wpx >= car_position and (light_wpx - car_position) <= 50:
                    if cls_light_wpx is None:
                        cls_light_wpx = light_wpx
                        light = lp
                    elif light_wpx < cls_light_wpx:
                        cls_light_wpx = light_wpx
                        light = lp
            #rospy.loginfo('car_position = %d, cls_light_wpx = %d', car_position, cls_light_wpx)

            if cls_light_wpx:
                min_dist = 0
                for slp in stop_line_positions:
                    #rospy.logwarn('slp x=%f, y=%f', slp[0], slp[1])
                    light_stop_pose = Pose()
                    light_stop_pose.position.x = slp[0]
                    light_stop_pose.position.y = slp[1]
                    light_stop_wp = self.get_closest_waypoint_birectional(light_stop_pose) 
                    # rospy.logwarn('light_stop_wp = %d', light_stop_wp)
                    dist = abs(cls_light_wpx - light_stop_wp)
                    if min_dist == 0:
                        min_dist = dist
                        cls_light_stop_wpx = light_stop_wp
                    elif dist < min_dist:
                        min_dist = dist
                        cls_light_stop_wpx = light_stop_wp
            # rospy.logwarn('cls_light_stop_wpx = %d', cls_light_stop_wpx)


        if light:
            state = self.get_light_state(light)

            if state == 4:
                color = 'Unknown'
            elif state == 2:
                color = 'Green'
            elif state == 1:
                color = 'Yellow'
            elif state == 0:
                color = 'Red'

            rospy.loginfo('Car Index: %d, Closest Traffic Light Index: %d, Traffic Light State: %s', car_position, cls_light_wpx, color)
            return cls_light_stop_wpx, state

        rospy.loginfo('Traffic Light State: %s', 'Unknown')
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
