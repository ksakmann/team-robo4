#!/usr/bin/env python

import math
import tf

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller.msg import ControlDiag
from speed_controller import SpeedController
from yaw_rate_controller import YawRateController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

        rospy.logdebug('DBW Initialization')

        vehicle_mass    = rospy.get_param('~vehicle_mass'   , 1736.35)
        fuel_capacity   = rospy.get_param('~fuel_capacity'  , 13.5)
        brake_deadband  = rospy.get_param('~brake_deadband' , .1)
        decel_limit     = rospy.get_param('~decel_limit'    , -5)
        accel_limit     = rospy.get_param('~accel_limit'    , 1.)
        wheel_radius    = rospy.get_param('~wheel_radius'   , 0.2413)
        wheel_base      = rospy.get_param('~wheel_base'     , 2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio'    , 14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel'  , 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.speed = 0
        self.speed_demand = 0
        self.yaw_rate = 0
        self.yaw_rate_demand = 0
        self.rate = 50 # DBW node rate (Hz)

	self.dbw_enabled = False

        # Publications
        self.steer_pub              = rospy.Publisher('/vehicle/steering_cmd' , SteeringCmd, queue_size=1)
        self.throttle_pub           = rospy.Publisher('/vehicle/throttle_cmd' , ThrottleCmd, queue_size=1)
        self.brake_pub              = rospy.Publisher('/vehicle/brake_cmd'    , BrakeCmd   , queue_size=1)
        self.speed_control_diag_pub = rospy.Publisher('/speed_control_diag'   , ControlDiag, queue_size=1)

        # Subscriptions
        self.dbw_enabled_sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool        , self.dbw_enabled_cb)
        self.speed_sub       = rospy.Subscriber('/current_velocity'   , TwistStamped, self.speed_cb)
        self.twist_cmd_sub   = rospy.Subscriber('/twist_cmd'          , TwistStamped, self.twist_cmd_cb)

        self.speed_controller    = SpeedController()
        self.yaw_rate_controller = YawRateController(max_steer_angle, steer_ratio, wheel_base)

        self.loop()

    def loop(self):

        rate = rospy.Rate(self.rate) # 50Hz

        rospy.logdebug('DBW Rate set')

        while not rospy.is_shutdown():
            # TODO Can we assume the sampling time is constant? Or do we need to calculated the time elapsed since last exec?
            dt = 1./self.rate
            throttle, brake = self.speed_controller.control(self.speed_demand, self.speed, dt)
            steer = self.yaw_rate_controller.control(self.speed, self.yaw_rate_demand)

            if self.dbw_enabled:
                rospy.loginfo('Speed Demand: %f, Speed Actual: %f', self.speed_demand, self.speed)
                rospy.loginfo('Yaw Rate Demand: %f, Yaw Rate Actual: %f', self.yaw_rate_demand, self.yaw_rate)
                self.publish(throttle, brake, steer)
            else:
                self.speed_controller.reset()

            # Diagnotics
            speed_diag           = ControlDiag()
            speed_diag.reference = self.speed_demand
            speed_diag.actuator  = self.speed_controller.pid.actuator
            speed_diag.error     = self.speed_controller.pid.error
            speed_diag.i_error   = self.speed_controller.pid.i_error
            speed_diag.d_error   = self.speed_controller.pid.d_error

            self.speed_control_diag_pub.publish(speed_diag)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        
        # Take care not to publish both throttle and brake
        if throttle > 0:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)

        if self.speed > 0.1:
            scmd = SteeringCmd()
            scmd.enable = True
            scmd.steering_wheel_angle_cmd = steer
            self.steer_pub.publish(scmd)
            rospy.loginfo('steer cmd: %s', steer)

        if brake > 0:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_PERCENT
            # It's uncertain what these units should be. It looks like
            # CMD_PERCENT is 0-1 for ThrottleCmd but 0-100 for BrakeCmd
            # Further it looks like 100% is not a lot of braking at all
            # So I've doubled that amount here to have a moderate amount 
            # of braking as observed in the simulator
            bcmd.pedal_cmd = brake * 200
            bcmd.boo_cmd = True
            self.brake_pub.publish(bcmd)

    def speed_cb(self, msg):
        self.speed = msg.twist.linear.x
        self.yaw_rate = msg.twist.angular.z        

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
    
    def twist_cmd_cb(self, msg):
        self.speed_demand = msg.twist.linear.x # m/s
        self.yaw_rate_demand = msg.twist.angular.z # rad/s

    
if __name__ == '__main__':
    try:
        DBWNode()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start dbw node.')
