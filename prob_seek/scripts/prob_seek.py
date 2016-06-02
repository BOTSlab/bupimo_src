#!/usr/bin/env python
"""
Implementation of ProbSeek algorithm described in the Cache Consensus paper.

Andrew Vardy
"""

import rospy, math
from bupimo_msgs.msg import Puck
from bupimo_msgs.msg import PuckArray
from bupimo_msgs.msg import Cluster
from bupimo_msgs.msg import ClusterArray
from bupimo_msgs.msg import ZumoProximities
from bupimo_msgs.msg import ObstacleArray
from geometry_msgs.msg import Twist

from bupimo_utils.pucks_and_clusters import get_puck_distance
from bupimo_utils.pucks_and_clusters import get_smallest_cluster
from bupimo_utils.pucks_and_clusters import get_closest_puck_in_cluster
from bupimo_utils.pucks_and_clusters import get_closest_puck
from bupimo_utils.pucks_and_clusters import get_closest_puck_to_puck
from bupimo_utils.pucks_and_clusters import get_largest_cluster_of_type
from bupimo_utils.pucks_and_clusters import get_closest_puck_distance
from bupimo_utils.movements import *

class ProbSeek:

    def __init__(self):
        rospy.init_node('prob_seek')

        # Initialize variables
        self.state = "PU_SCAN"
        self.puck_in_gripper = False
        self.carried_type = None
        self.target_puck = None
        self.current_step = 0
        self.state_start_step = 0
        self.random_turn_dir = 1
        self.random_turn_time = 0
        self.obstacle_array_msg = None

        # Parameters (move to ROS parameter server?)
        self.PICK_UP_TIME = 100
        self.TARGET_FINAL_TIME = 10
        self.PLACEMENT_TIME = 40
        self.PUSH_IN_TIME = 1
        self.BACKUP_TIME = 5
        self.CLUSTER_CONTACT_DISTANCE = 0.12
        self.MIN_TURN_TIME = 5
        self.MAX_TURN_TIME = 10
        self.K1 = 1.0
        self.K2 = 2.0

        # Publish to 'cmd_vel'
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Subscriptions
        rospy.Subscriber('clusters', ClusterArray, self.clusters_callback)
        rospy.Subscriber('zumo_prox', ZumoProximities, self.prox_callback)
        rospy.Subscriber('obstacles', ObstacleArray, self.obstacles_callback)

        rospy.on_shutdown(self.shutdown_handler)

    def prox_callback(self, prox_msg):
        """The front prox. sensor will tell us if a puck is in the gripper."""
        self.puck_in_gripper = prox_msg.front <= 8

    def obstacles_callback(self, obstacle_array_msg):
        self.obstacle_array_msg = obstacle_array_msg

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
            
    def accept_as_deposit_cluster(self, cluster):
        """Accept cluster with probability given by cluster size."""
        if cluster == None:
            return False
        n = len(cluster.array.pucks)
        # Deneubourg et al formula
        prob = (n / (self.K2 + n))**2
        print("Cluster! type: " + str(cluster.type) + ", n: " + str(n) + \
              ", probability of deposit: " + str(prob))
        accept = (random.random() < prob)
        print("\taccept: " + str(accept))

        return accept
            
    def set_new_pickup_target_from_cluster(self, cluster):
        closest_puck = get_closest_puck_in_cluster(cluster)
        assert closest_puck != None
        self.target_puck = closest_puck
        self.carried_type = closest_puck.type
        print("carried type: " + str(self.carried_type))

    def maintain_pickup_target(self, cluster_array_msg):
        candidate = get_closest_puck_to_puck(cluster_array_msg, \
                                             self.target_puck, 0.05)
        if candidate == None or candidate.type != self.carried_type:
            # If there is no candidate or the type has changed then we will
            # indicate that tracking has failed by setting the target to None.
            self.target_puck = None
        else:
            self.target_puck = candidate

    def initiate_random_turn(self):
        # Set the turn direction as either -1 or 1
        self.random_turn_dir = random.randint(0,1)
        if self.random_turn_dir == 0:
            self.random_turn_dir = -1

        self.random_turn_time = random.randint(self.MIN_TURN_TIME, 
                                               self.MAX_TURN_TIME)

    def clusters_callback(self, cluster_array_msg):
        """This callback is the primary control method of the class."""

        self.current_step = cluster_array_msg.header.seq
        time_in_state = self.current_step - self.state_start_step

        ######################################################################
        # Handle state transitions
        ######################################################################
        #print(self.state)
        if self.state == "PU_SCAN":
            if self.puck_in_gripper:
                if self.carried_type == None:
                    self.transition("DE_BACKUP", "Get rid of strange puck!")
                else:
                    self.transition("DE_SCAN", "Somehow acquired puck!")
            else:
                smallest_clust = get_smallest_cluster(cluster_array_msg)
                # Accept this as the pickup cluster with some chance
                accept = self.accept_as_pickup_cluster(smallest_clust)
                if accept:
                    self.set_new_pickup_target_from_cluster(smallest_clust)
                    if self.target_puck != None:
                        self.transition("PU_TARGET", "Pick-up target acquired")

        elif self.state == "PU_TARGET":
            if self.puck_in_gripper:
                self.transition("DE_SCAN", "Pick-up succeeded")
            else:
                self.maintain_pickup_target(cluster_array_msg)
                if self.target_puck == None:
                    self.transition("PU_SCAN", "Lost target")
                elif time_in_state > self.PICK_UP_TIME:
                    self.transition("PU_SCAN", "Time out")
                elif get_puck_distance(self.target_puck) < 0.1:
                    self.transition("PU_TARGET_CLOSE", "Puck is close")

        elif self.state == "PU_TARGET_CLOSE":
            if self.puck_in_gripper:
                self.transition("DE_SCAN", "Pick-up succeeded early")
            else:
                self.maintain_pickup_target(cluster_array_msg)
                if self.target_puck == None:
                    self.transition("PU_TARGET_FINAL", \
                               "Target puck invisible, but almost in")
                elif time_in_state > self.TARGET_FINAL_TIME:
                    self.transition("PU_SCAN", "Time out")

        elif self.state == "PU_TARGET_FINAL":
            if self.puck_in_gripper:
                self.transition("DE_SCAN", "Pick-up succeeded")
                
        elif self.state == "DE_SCAN":
#            if not self.puck_in_gripper:
#                self.transition("PU_SCAN", "Somehow lost puck!")
#            else:
# INDENT REST OF BLOCK IF ABOVE UNCOMMENTED
            #print("DE_SCAN: carried type: " + str(self.carried_type))
            largest_clust = get_largest_cluster_of_type(cluster_array_msg,
                                                        self.carried_type)
            # Accept this as the deposit cluster with some chance
            accept = self.accept_as_deposit_cluster(largest_clust)
            if accept:
                self.target_puck = get_closest_puck_in_cluster(largest_clust)
                if self.target_puck != None:
                    self.transition("DE_TARGET", "Deposit target acquired")

        elif self.state == "DE_TARGET":
            if not self.puck_in_gripper:
                self.transition("PU_SCAN", "Somehow lost puck!")
            else:
                # Is the closest puck close enough for us to say we have hit
                # the target cluster (hard to tell if it is that cluster)
                if get_closest_puck_distance(cluster_array_msg) < \
                                                self.CLUSTER_CONTACT_DISTANCE:
                    self.transition("DE_PUSH", "Contacted cluster")
                else:
                    self.target_puck = get_closest_puck_to_puck(
                                                            cluster_array_msg,
                                                            self.target_puck,
                                                            0.1)
                    if self.target_puck == None:
                        self.transition("DE_SCAN", "Lost target")
                    elif time_in_state > self.PLACEMENT_TIME:
                        self.transition("SE_SCAN", "Time out")

        elif self.state == "DE_PUSH":
            if time_in_state > self.PUSH_IN_TIME:
                self.transition("DE_BACKUP", "")
            
        elif self.state == "DE_BACKUP":
            self.carried_type = None
            print("carried type: " + str(self.carried_type))
            if time_in_state > self.BACKUP_TIME:
                self.transition("DE_TURN", "")
                self.initiate_random_turn()

        elif self.state == "DE_TURN":
            if time_in_state > self.random_turn_time:
                self.transition("PU_SCAN", "Deposit cycle complete")

        else:
            sys.exit("Unknown state")
            

        ######################################################################
        # Set velocity based on state.
        ######################################################################
        twist = None
        if self.state == "PU_SCAN" or self.state == "DE_SCAN":
            twist = wander_while_avoiding_obs_pucks(self.obstacle_array_msg, \
                                                    cluster_array_msg)

        elif self.state == "PU_TARGET" or \
             self.state == "PU_TARGET_CLOSE" or \
             self.state == "DE_TARGET":
            twist = move_to_puck(self.target_puck)

        elif self.state == "DE_PUSH" or self.state == "PU_TARGET_FINAL":
            twist = forwards()

        elif self.state == "DE_BACKUP":
            twist = backwards()

        elif self.state == "DE_TURN":
            twist = turn(self.random_turn_dir)

        else:
            sys.exit("Unknown state")

        self.cmd_vel_publisher.publish(twist)

    def shutdown_handler(self):
        # Stop the robot
        self.cmd_vel_publisher.publish(Twist())

if __name__ == '__main__':
    prob_seek = ProbSeek()
    rospy.spin()
