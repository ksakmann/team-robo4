# Waypoint Updater Node

## Overview

The waypoint updater node is responsible for processing the static basewaypoint nodes and retrieving a specific set of waypoint nodes representing the immediate future set of horizon nodes. Furthermore, these nodes carry respective target velocities that account for any red traffic lights that may have been detected.

The ROS parameter `lookahead_wps` controls how many waypoints are included in the horizon set. If a red traffic light is published to the `/traffic_waypoint` topic a velocity profile is calculated in order to slow down at a specified linear deceleration rate and come to a complete stop at the specified waypoint. The linear deceleration rate is specified in the parameter `nominal_deccel` and the stopping distance includes a buffer which can be adjusted by the parameter `stopping_distance_buffer`. As an example if you'd like the vehicle to stop braking sooner you can make the `nominal_deccel` value lower which will calculate the velocity profile with a lower decceleration rate and thus longer braking distance. Both parameters are in metric units. 

Note that if the vehicle is incapable of coming to a complete stop before the traffic light, it will abort the braking maneuver and return to the nominal target speed. If this is happening often the pratical thing to do is to increase the buffer distance `stopping_distance_buffer` parameter such that the margin of error is increased and the chances of stopping ahead of the traffic light are increased. 