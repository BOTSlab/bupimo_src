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
from obstacle_detector.msg import CastObstacleArray
from geometry_msgs.msg import Twist

from bupimo_utils.pucks_and_clusters import get_puck_distance
from bupimo_utils.pucks_and_clusters import get_closest_cluster
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
        #self.obstacle_array_msg = None
        self.castobstacle_array_msg = None
        self.nongripper_time = 0
        self.prox_below_threshold = False

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

        # To support pausing
        do_pause = False
        self.pause_interval = 25
        self.pause_movetime = 5 
        self.pause_counter = 0

        # Publish to 'cmd_vel'
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Subscriptions
        if do_pause:
            rospy.Subscriber('clusters', ClusterArray, self.pausing_callback)
        else:
            rospy.Subscriber('clusters', ClusterArray, self.working_callback)

        rospy.Subscriber('zumo_prox', ZumoProximities, self.prox_callback)
        #rospy.Subscriber('obstacles', ObstacleArray, self.obstacles_callback)
        rospy.Subscriber('castobstacles', CastObstacleArray, self.castobstacles_callback)

        rospy.on_shutdown(self.shutdown_handler)

    def prox_callback(self, prox_msg):
        """The front prox. sensor will tell us if a puck is in the gripper."""
#        print("prox_msg.front: " + str(prox_msg.front))
        self.prox_below_threshold = prox_msg.front <= 8

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
            
#    def set_new_pickup_target_from_cluster(self, cluster):
#        closest_puck = get_closest_puck_in_cluster(cluster)
#        assert closest_puck != None
#        self.target_puck = closest_puck

    def maintain_pickup_target(self, cluster_array_msg):
        candidate = get_closest_puck_to_puck(cluster_array_msg, \
                                             self.target_puck, 100000.10)
        if candidate == None or candidate.type != self.target_puck.type:
            # If there is no candidate or the type has changed then we will
            # indicate that tracking has failed by setting the target to None.
            return None
        else:
            return candidate

    def maintain_deposit_target(self, cluster_array_msg):
        candidate = get_closest_puck_to_puck(cluster_array_msg, \
                                             self.target_puck, 100000.05)
        if candidate == None or candidate.type != self.carried_type:
            # If there is no candidate or the type is not the carried type 
            # then we indicate that tracking has failed by setting the target
            # to None.
            return None
        else:
            return candidate

    def initiate_random_turn(self):
        # Set the turn direction as either -1 or 1
        self.random_turn_dir = random.randint(0,1)
        if self.random_turn_dir == 0:
            self.random_turn_dir = -1

        self.random_turn_time = random.randint(self.MIN_TURN_TIME, 
                                               self.MAX_TURN_TIME)

    def pausing_callback(self, cluster_array_msg):
        if (self.pause_counter % self.pause_interval) == 0:
            # Do the actual work
            working_callback(cluster_array_msg)

        elif (self.pause_counter % self.pause_interval) < self.pause_movetime:
            # Continue with previously published speed
            pass

        else:
            # Stop
            self.cmd_vel_publisher.publish(Twist())

        self.pause_counter += 1

    def working_callback(self, cluster_array_msg):
        """This callback is the primary control method of the class."""

        self.current_step = cluster_array_msg.header.seq
        time_in_state = self.current_step - self.state_start_step


        if self.prox_below_threshold: # Indicating "possibly" a puck
            self.nongripper_time = 0
        else:
            self.nongripper_time += 1

        self.puck_in_gripper = (self.nongripper_time < 10)
            

        ######################################################################
        # Handle state transitions
        ######################################################################
        print("state: " + self.state + ", time_in_state: " + str(time_in_state))
        if self.state == "PU_SCAN":
            self.carried_type = None
            if self.puck_in_gripper:
                #if self.carried_type == None:
                self.transition("DE_BACKUP", "Get rid of strange puck!")
                #else:
                #    self.transition("DE_SCAN", "Somehow acquired puck!")
            else:
                # We consider only the cluster that is closest.
                closest_c = get_closest_cluster(cluster_array_msg)
                # Accept this as the pickup cluster with some chance
                if self.accept_as_pickup_cluster(closest_c):
                    self.target_puck = get_closest_puck_in_cluster(closest_c)
                    if self.target_puck != None:
                        self.transition("PU_TARGET", "Pick-up target acquired")

        elif self.state == "PU_TARGET":
            print("target type: " + str(self.target_puck.type))

            if self.puck_in_gripper:
                self.transition("DE_SCAN", "Pick-up succeeded")
                self.carried_type = self.target_puck.type
                print("carried type: " + str(self.carried_type))
            else:
                print(" IN PU_TARGET : target == None " + str(self.target_puck == None))
                self.target_puck = self.maintain_pickup_target( \
                                                            cluster_array_msg)
                if self.target_puck == None:
                    self.transition("PU_SCAN", "Lost target")
                elif time_in_state > self.PICK_UP_TIME:
                    self.transition("PU_SCAN", "Time out")
                elif get_puck_distance(self.target_puck) < 0.1:
                    self.transition("PU_TARGET_CLOSE", "Puck is close")

        elif self.state == "PU_TARGET_CLOSE":
            if self.puck_in_gripper:
                self.transition("DE_SCAN", "Pick-up succeeded early")
                self.carried_type = self.target_puck.type
                print("carried type: " + str(self.carried_type))
            else:
                result = self.maintain_pickup_target(cluster_array_msg)
                if result == None:
                    self.transition("PU_TARGET_FINAL", \
                               "Target puck invisible, but almost in")

        elif self.state == "PU_TARGET_FINAL":
            if self.puck_in_gripper:
                self.transition("DE_SCAN", "Pick-up succeeded")
                self.carried_type = self.target_puck.type
                print("carried type: " + str(self.carried_type))
            elif time_in_state > self.TARGET_FINAL_TIME:
                self.transition("PU_SCAN", "Time out")
                
        elif self.state == "DE_SCAN":
            if self.puck_in_gripper and self.carried_type == None:
                self.transition("DE_BACKUP", "Get rid of strange puck!")
#            elif not self.puck_in_gripper:
#                self.transition("PU_SCAN", "Somehow lost puck!")
#                self.carried_type = None
            else:
                print("DE_SCAN: carried type: " + str(self.carried_type))
                closest_puck = get_closest_puck(cluster_array_msg)
                if closest_puck != None \
                    and closest_puck.type == self.carried_type \
                    and get_puck_distance(closest_puck) < \
                                                self.CLUSTER_CONTACT_DISTANCE:
                    self.transition("DE_PUSH", "Non-targeted cluster --- Ok!")
                else:
                    #big_c = get_largest_cluster_of_type(cluster_array_msg, \
                    #                                    self.carried_type)
                    # We consider only the cluster that is closest.
                    clust = get_closest_cluster(cluster_array_msg)
                    # Accept this as the deposit cluster with some chance, but
                    # only if its the right type.
                    if clust != None \
                        and clust.type == self.carried_type \
                        and self.accept_as_deposit_cluster(clust):
                        self.target_puck = get_closest_puck_in_cluster(clust)
                        if self.target_puck != None:
                            self.transition("DE_TARGET", "Deposit target acq'd")

        elif self.state == "DE_TARGET":
            if not self.puck_in_gripper:
                self.transition("PU_SCAN", "Somehow lost puck!")
            else:
                self.target_puck = self.maintain_deposit_target( \
                                                            cluster_array_msg)
                if self.target_puck == None:
                    self.transition("DE_SCAN", "Lost target")
                elif get_puck_distance(self.target_puck) < \
                                                self.CLUSTER_CONTACT_DISTANCE:
                    self.transition("DE_PUSH", "Contacted cluster")
                elif time_in_state > self.PLACEMENT_TIME:
                    self.transition("DE_SCAN", "Time out")

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
            #twist = wander_while_avoiding_obs_pucks(self.obstacle_array_msg, \
            #                                        cluster_array_msg)
            twist = wander_while_avoiding_castobs(self.castobstacle_array_msg)

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
