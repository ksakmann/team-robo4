# Traffic Light Detector Node

## Overview

The traffic light detector node is responsible for detecting traffic lights ahead of the vehicle. In the case of a red light detection, the event along with the location of the nearest traffic light stopping zone (ahead of the vehcile) is published to the `/traffic_waypoint` topic.

In order to smooth intermittent mis-classification errors, the traffig light color status is only recognized once 3 consecutive classicifactions return the same state. 