# Drive-By-Wire Node

## Overview

The DBW node is responsible for publishing throttle, brake and steering set-point values for the car's lower level control system to execute. Most of the node has been kept as simple as possible in order to help with the debugging and feature development that is likely to occur as real-world testing commences. The controller was developed in a simulator with a target operating velocity of 10 MPH.

The throttle and brake set-points are calulcated using a simple feedback proportional controller. The controller acts on the velocity error between the demanded and actual vehicle velocity.

The steering setpoint is calculated using a simple feedforward controller based on simple stead-state bicycle model of a vehicle. The controller acts on a demanded angular velocity.

The node runs at a rate of 50 Hz.