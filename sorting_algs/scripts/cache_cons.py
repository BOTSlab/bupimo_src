#!/usr/bin/env python
"""
Implements a variant of the Cache Consensus algorithm described in the following paper:

A. Vardy, G. Vorobyev, W. Banzhaf
Cache Consensus: Rapid Object Sorting by a Robotic Swarm
Swarm Intelligence, 8 (1), pp. 61-87, 2014.

This algorithm differs from the one described above in (at least) the following
significant ways:
    1. Using visual homing as opposed to having direct access to positions.
Assuming we can only obtain the bearing towards remembered locations.
    2. We cannot really recognize a cluster as the cache cluster or not.  We just home towards the remembered point and deposit whenever we bump into a puck of the right type.  The only other way out of this state (HOMING) would be through a timeout.
    3. Any visible cluster that is larger than the previous cache cluster will be taken as the new cache cluster (for that type).  But we have to make contact with the cluster before storing the goal image.  So there is a new state called VISIT_NEW_CACHE to achieve this.

Andrew Vardy
"""

import rospy, math, sys
from bupimo_msgs.msg import Puck
from bupimo_msgs.msg import PuckArray
from bupimo_msgs.msg import Cluster
from bupimo_msgs.msg import ClusterArray
from bupimo_msgs.msg import ZumoProximities
from bupimo_msgs.msg import ObstacleArray
from scale_space_homing_service.msg import CastObstacleArray
from scale_space_homing_service.srv import SetGoalLocation
from scale_space_homing_service.srv import GetBearingForGoal
from geometry_msgs.msg import Twist

from bupimo_utils.pucks_and_clusters import get_puck_distance
from bupimo_utils.pucks_and_clusters import get_smallest_cluster
from bupimo_utils.pucks_and_clusters import get_closest_puck_in_cluster
from bupimo_utils.pucks_and_clusters import get_closest_puck
from bupimo_utils.pucks_and_clusters import get_closest_puck_to_puck
from bupimo_utils.pucks_and_clusters import get_largest_cluster_of_type
from bupimo_utils.pucks_and_clusters import get_closest_puck_distance
from bupimo_utils.movements import *

class CacheCons:

    def __init__(self):
        rospy.init_node('cache_cons')

        # Initialize variables
        self.state = "PU_SCAN"
        self.puck_in_gripper = False
        self.carried_type = None
        self.target_puck = None
        self.current_step = 0
        self.state_start_step = 0
        #self.obstacle_array_msg = None
        self.castobstacle_array_msg = None

        # Dictionary of cache sizes with puck type as the key.  An empty entry
        # means no pucks of that type have yet been seen.  This is all we need
        # to store here with respect to caches --- the goal images reside in
        # the provider of the 'set_goal' and the 'get_bearing' services.
        self.cache_sizes = {}

        # Parameters (move to ROS parameter server?)
        self.PICK_UP_TIME = 100
        self.TARGET_FINAL_TIME = 10
        self.HOMING_TIME = 100
        self.PUSH_IN_TIME = 1
        self.BACKUP_TIME = 5
        self.EXILE_TIME = 10
        self.CLUSTER_CONTACT_DISTANCE = 0.12
        self.K1 = 1.0

        # Setup the services necessary for homing.
        rospy.wait_for_service('set_goal_location')
        self.set_goal = rospy.ServiceProxy('set_goal_location', SetGoalLocation)
        rospy.wait_for_service('get_bearing_for_goal')
        self.get_bearing = rospy.ServiceProxy('get_bearing_for_goal', \
                                              GetBearingForGoal)

        # Publish to 'cmd_vel'
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Subscriptions
        rospy.Subscriber('clusters', ClusterArray, self.clusters_callback)
        rospy.Subscriber('zumo_prox', ZumoProximities, self.prox_callback)
        #rospy.Subscriber('obstacles', ObstacleArray, self.obstacles_callback)
        rospy.Subscriber('castobstacles', CastObstacleArray, self.castobstacles_callback)

        rospy.on_shutdown(self.shutdown_handler)

    def prox_callback(self, prox_msg):
        """The front prox. sensor will tell us if a puck is in the gripper."""
        self.puck_in_gripper = prox_msg.front <= 8

#    def obstacles_callback(self, obstacle_array_msg):
#        self.obstacle_array_msg = obstacle_array_msg

    def castobstacles_callback(self, castobstacle_array_msg):
        self.castobstacle_array_msg = castobstacle_array_msg

    def transition(self, new_state, message):
        self.state = new_state
        self.state_start_step = self.current_step
        print("transition: " + self.state + " --- " + message)

    def accept_as_pickup_cluster(self, cluster):
        """Accept cluster with probability given by cluster size."""
        if cluster == None:
            return False
        n = len(cluster.array.pucks)
        # Deneubourg et al formula
        prob = (self.K1 / (self.K1 + n))**2
        print("Cluster! type: " + str(cluster.type) + ", n: " + str(n) + \
              ", probability of pickup: " + str(prob))
        accept = (random.random() < prob)
        print("\taccept: " + str(accept))

        return accept
            
    def set_new_pickup_target_from_cluster(self, cluster):
        closest_puck = get_closest_puck_in_cluster(cluster)
        assert closest_puck != None
        self.target_puck = closest_puck
        self.carried_type = closest_puck.type
        print("carried type: " + str(self.carried_type))

    def maintain_pickup_or_visit_target(self, cluster_array_msg):
        """Used for tracking targets to pickup or to visit."""
        candidate = get_closest_puck_to_puck(cluster_array_msg, \
                                             self.target_puck, 0.05)
        if candidate == None or candidate.type != self.target_puck:
            # If there is no candidate or the type has changed then we will
            # indicate that tracking has failed by setting the target to None.
            self.target_puck = None
        else:
            self.target_puck = candidate

    def clusters_callback(self, cluster_array_msg):
        """This callback is the primary control method of the class."""

        self.current_step = cluster_array_msg.header.seq
        time_in_state = self.current_step - self.state_start_step
        #print(self.state)

        ######################################################################
        # Handle transition to VISIT_NEW_CACHE which can happen from any state
        ######################################################################
        cluster_to_visit = None
        for cluster in cluster_array_msg.clusters:
            size = len(cluster.array.pucks)
            if (cluster.type not in self.cache_sizes) or \
               (size > self.cache_sizes[cluster.type]):
                self.transition("VISIT_NEW_CACHE", "Touch the new cache")
                self.cache_sizes[cluster.type] = size
                cluster_to_visit = cluster

        ######################################################################
        # Handle other state transitions
        ######################################################################
        if self.state == "VISIT_NEW_CACHE":
            if cluster_to_visit != None:
                self.target_puck = get_closest_puck_in_cluster(cluster_to_visit)
            else:
                self.maintain_pickup_or_visit_target(cluster_array_msg)
            if self.target_puck == None:
                self.transition("PU_SCAN", "Cache disappeared prior to visit")
            elif get_puck_distance(self.target_puck) < \
                                            self.CLUSTER_CONTACT_DISTANCE:
                self.transition("DE_BACKUP", "Established new cache")
                # Capture goal image!
                self.set_goal(self.target_puck.type)
                self.target_puck = None

        if self.state == "PU_SCAN":
            if self.puck_in_gripper:
                if self.carried_type == None:
                    self.transition("DE_BACKUP", "Get rid of strange puck!")
                else:
                    self.transition("HOMING", "Somehow acquired puck!")
            else:
                smallest_clust = get_smallest_cluster(cluster_array_msg)
                # Accept this as the pickup cluster with some chance
                if self.accept_as_pickup_cluster(smallest_clust):
                    self.set_new_pickup_target_from_cluster(smallest_clust)
                    if self.target_puck != None:
                        self.transition("PU_TARGET", "Pick-up target acquired")

        elif self.state == "PU_TARGET":
            if self.puck_in_gripper:
                self.transition("DE_SCAN", "Pick-up succeeded")
            else:
                self.maintain_pickup_or_visit_target(cluster_array_msg)
                if self.target_puck == None:
                    self.transition("PU_SCAN", "Lost target")
                elif time_in_state > self.PICK_UP_TIME:
                    self.transition("PU_SCAN", "Time out")
                elif get_puck_distance(self.target_puck) < 0.1:
                    self.transition("PU_TARGET_CLOSE", "Puck is close")

        elif self.state == "PU_TARGET_CLOSE":
            if self.puck_in_gripper:
                self.transition("HOMING", "Pick-up succeeded early")
            else:
                self.maintain_pickup_or_visit_target(cluster_array_msg)
                if self.target_puck == None:
                    self.transition("PU_TARGET_FINAL", \
                               "Target puck invisible, but almost in")

        elif self.state == "PU_TARGET_FINAL":
            if self.puck_in_gripper:
                self.transition("HOMING", "Pick-up succeeded")
            elif time_in_state > self.TARGET_FINAL_TIME:
                self.transition("PU_SCAN", "Time out")
                
        elif self.state == "HOMING":
            if not self.puck_in_gripper:
                self.transition("PU_SCAN", "Somehow lost puck!")
            else:
                closest_puck = get_closest_puck(cluster_array_msg)
                if closest_puck != None \
                    and closest_puck.type == self.carried_type \
                    and get_puck_distance(closest_puck) < \
                                                self.CLUSTER_CONTACT_DISTANCE:
                    self.transition("DE_PUSH", "Contacted cluster")
                elif time_in_state > self.HOMING_TIME:
                    self.transition("DE_BACKUP", "Time out")

        elif self.state == "DE_PUSH":
            if time_in_state > self.PUSH_IN_TIME:
                self.transition("DE_BACKUP", "")
            
        elif self.state == "DE_BACKUP":
            self.carried_type = None
            print("carried type: " + str(self.carried_type))
            if time_in_state > self.BACKUP_TIME:
                self.transition("EXILE", "")

        elif self.state == "EXILE":
            if time_in_state > self.EXILE_TIME:
                self.transition("PU_SCAN", "Deposit cycle complete")

        else:
            sys.exit("Unknown state")
            

        ######################################################################
        # Set velocity based on state.
        ######################################################################
        twist = None
        if self.state == "VISIT_NEW_CACHE":
            closest_puck = get_closest_puck_in_cluster(cluster)
            assert closest_puck != None
            twist = move_to_puck(closest_puck)

        if self.state == "PU_SCAN":
            #twist = wander_while_avoiding_obs_pucks(self.obstacle_array_msg, \
            #                                        cluster_array_msg)
            twist = wander_while_avoiding_castobs(self.castobstacle_array_msg)

        elif self.state == "PU_TARGET" or \
             self.state == "PU_TARGET_CLOSE":
            twist = move_to_puck(self.target_puck)

        elif self.state == "HOMING":
            response = self.get_bearing(self.carried_type)
            if response.is_valid:
                twist = move_towards_bearing(response.bearing)
            else
                print("Bearing invalid: wandering instead")
                twist = wander_while_avoiding_castobs( \
                                                    self.castobstacle_array_msg)

        elif self.state == "DE_PUSH" or self.state == "PU_TARGET_FINAL":
            twist = forwards()

        elif self.state == "DE_BACKUP":
            twist = backwards()

        elif self.state == "EXILE":
            # Go away from the "goal"
            response = self.get_bearing(self.carried_type) + math.pi
            if response.is_valid:
                twist = move_towards_bearing(bearing)
            else:
                print("Bearing invalid: wandering instead")
                twist = wander_while_avoiding_castobs( \
                                                    self.castobstacle_array_msg)

        else:
            sys.exit("Unknown state")

        self.cmd_vel_publisher.publish(twist)

    def shutdown_handler(self):
        # Stop the robot
        self.cmd_vel_publisher.publish(Twist())

if __name__ == '__main__':
    cache_cons = CacheCons()
    rospy.spin()
