#!/usr/bin/env python  
"""
Uses RVO2 collision avoidance simulation to avoid obstacles inferred from laser
data.

Andrew Vardy
"""

import rospy, rvo2
from math import *
from rvo2_laser.srv import Activate
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist

class RVOAvoider:

    NUMBER_PREF_VELS = 11
    ANGLE_MIN = -pi/2.0
    ANGLE_MAX = pi/2.0
    ROBOT_RADIUS = 0.05
    MAX_LINEAR_SPEED = 0.5
    FULL_SEARCH_MAG_THRESHOLD = 0.1
    SIM_STEPS = 5

    def __init__(self):

        # Angles of preferred velocities that will be tested each iteration.
        angles = []
        angle_delta = (self.ANGLE_MAX - self.ANGLE_MIN) / \
                      (self.NUMBER_PREF_VELS - 1)
        for i in range(self.NUMBER_PREF_VELS):
            angles.append(self.ANGLE_MIN + i * angle_delta)
        self.pref_vels = []
        for angle in angles:
            self.pref_vels.append((self.MAX_LINEAR_SPEED * cos(angle), \
                                   self.MAX_LINEAR_SPEED * sin(angle)))

        self.last_index = angles.index(0)
        self.last_mag = float('inf')

        rospy.init_node('rvo2_laser')

        # Create a publisher so that we can output command velocities.
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.odom_twist = Twist()

        self.service = rospy.Service('activate_avoider', Activate, \
                                     self.handle_activate)

        self.active = True

        # Subscribe to odom
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Subscribe to scan
        rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def handle_activate(self, request):
        self.active = request.set_active
        return ActivateResponse()

    def odom_callback(self, odom):
        self.odom_twist = odom.twist.twist

    def scan_callback(self, scan):
        if not self.active:
            return

        # We seem to have to create a new simulator object each time because
        # otherwise it would contain the obstacles from the last time step.
        # If there was a 'removeObstacle' method it would be a bit nicer.
        sim = rvo2.PyRVOSimulator(1/60., # Time step
                                  1.5,   # neighborDist
                                  5,     # maxNeighbors
                                  1.5,   # timeHorizon (other agents)
                                  2.0,   #2     # timeHorizon (obstacles)
                                  self.ROBOT_RADIUS,   # agent radius
                                  self.MAX_LINEAR_SPEED)     # agent max speed
        agent = sim.addAgent((0, 0))

        # Add scan points as obstacles for the RVO simulator
        n = len(scan.ranges)
        points = []
        for i in range(0, n):
            r = scan.ranges[i]
            if not (r == float('inf') or isnan(r) or r < self.ROBOT_RADIUS):
                theta = scan.angle_min + i * scan.angle_increment
                rho = self.ROBOT_RADIUS + r
                points.append((rho*cos(theta), rho*sin(theta)))

        # The scan points will be treated together as a single "negative"
        # obstacle, with vertices specified in CW order.  This requires the
        # list of points in reverse.
        points.reverse()
        sim.addObstacle(points)
        sim.processObstacles()

        # To prevent oscillation we will generally just test the preferred
        # velocities in the immediate neighbourhood (within the pref_vels list)
        # of the preferred velocity chosen last time.
        if self.last_mag < self.FULL_SEARCH_MAG_THRESHOLD:
            # Last time the magnitude of the chosen velocity was very low.
            # Do a full search over the preferred velocities.
            start_index = 0
            stop_index = self.NUMBER_PREF_VELS - 1
        elif self.last_index == 0:
            start_index = 0
            stop_index = 1
        elif self.last_index == len(self.pref_vels)-1:
            start_index = self.NUMBER_PREF_VELS - 2
            stop_index = self.NUMBER_PREF_VELS - 1
        else:
            # This is the general case.
            start_index = self.last_index - 1
            stop_index = self.last_index + 1

        print ""
        highest_mag = 0
        chosen_vel = None
        chosen_index = None
        for i in range(start_index, stop_index+1):
            pref_vel = self.pref_vels[i]

            # Initializing from scratch each time
            sim.setAgentPosition(agent, (0, 0))
            # Set the current velocity.  TBD: What about angular velocity?
            sim.setAgentVelocity(agent, (self.odom_twist.linear.x, 0))
            sim.setAgentPrefVelocity(agent, pref_vel)
            
            for j in range(self.SIM_STEPS):
                sim.doStep()

            (vx, vy) = sim.getAgentVelocity(0)
            print "vel: {}, {}".format(vx, vy)

            mag = sqrt(vx*vx + vy*vy)
            if mag > highest_mag:
                highest_mag = mag
                chosen_vel = (vx, vy)
                chosen_index = i

        self.last_index = chosen_index
        self.last_mag = highest_mag
        print "highest_mag: {}".format(highest_mag)
        print "chosen_vel: {}, {}".format(chosen_vel[0], chosen_vel[1])

        # Now treat (vx, vy) as the goal and apply the simple control law
        twist = Twist()
        twist.linear.x = 0.25 * chosen_vel[0]
        twist.angular.z = 2.5 * chosen_vel[1]
        self.cmd_vel_publisher.publish(twist)

if __name__ == '__main__':
    
    avoider = RVOAvoider()
    rospy.spin()
