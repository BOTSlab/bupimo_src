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
from bupimo_msgs.msg import ZumoData
from rvo2_laser.srv import Activate
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

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
        self.carried_type = None
        self.target_type = None
        self.current_step = 0
        self.state_start_step = 0
        self.random_turn_dir = 1
        self.random_turn_time = 0
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

        # Publish to 'servo_control'
        self.servo_publisher = rospy.Publisher('servo_control', Bool, \
                                               queue_size=1)

        # Subscriptions
        if do_pause:
            rospy.Subscriber('clusters', ClusterArray, self.pausing_callback)
        else:
            rospy.Subscriber('clusters', ClusterArray, self.working_callback)

        rospy.Subscriber('zumo_data', ZumoData, self.prox_callback)

        rospy.wait_for_service('activate_avoider')
        self.activate_avoider = rospy.ServiceProxy('activate_avoider', Activate)
        # Since we start in PU_SCAN, we want to wander right away.
        #self.activate_avoider(True)

        rospy.on_shutdown(self.shutdown_handler)

    def prox_callback(self, zumo_msg):
        """The front prox. sensor will tell us if a puck is in the gripper."""
#        print("prox_msg.front: " + str(prox_msg.front))
        self.prox_below_threshold = zumo_msg.frontProximity <= 8

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

        ######################################################################
        # Handle state transitions
        ######################################################################
        print("state: " + self.state + ", time_in_state: " + str(time_in_state))
        if self.state == "PU_SCAN":
            self.target_type = None
            self.carried_type = None
            if self.prox_below_threshold:
                self.transition("DE_BACKUP", "Get rid of strange puck!")
            else:
                # We consider only the cluster that is closest.
                closest_c = get_closest_cluster(cluster_array_msg)
                # Accept this as the pickup cluster with some chance
                if self.accept_as_pickup_cluster(closest_c):
                    self.target_type = closest_c.type
                    self.transition("PU_TARGET", "Pick-up target acquired")

        elif self.state == "PU_TARGET":
            print("target_type: " + str(self.target_type))

            if self.prox_below_threshold:
                self.transition("DE_SCAN", "Pick-up succeeded")
                self.carried_type = self.target_type
                print("carried type: " + str(self.carried_type))
            else:
                if time_in_state > self.PICK_UP_TIME:
                    self.transition("DE_BACKUP", "Time out")

        elif self.state == "DE_SCAN":
            if self.prox_below_threshold and self.carried_type == None:
                self.transition("DE_BACKUP", "Get rid of strange puck!")
            else:
                print("DE_SCAN: carried type: " + str(self.carried_type))
                closest_puck = get_closest_puck(cluster_array_msg)
                if closest_puck != None \
                    and closest_puck.type == self.carried_type \
                    and get_puck_distance(closest_puck) < \
                                                self.CLUSTER_CONTACT_DISTANCE:
                    self.transition("DE_PUSH", "Contacted cluster --- Ok!")
                else:
                    # We consider only the cluster that is closest.
                    clust = get_closest_cluster(cluster_array_msg)
                    # Accept this as the deposit cluster with some chance, but
                    # only if its the right type.
                    if clust != None \
                        and clust.type == self.carried_type \
                        and self.accept_as_deposit_cluster(clust):
                        self.transition("DE_TARGET", "Deposit target acq'd")

        elif self.state == "DE_TARGET":
            if self.prox_below_threshold and self.carried_type == None:
                self.transition("DE_BACKUP", "Get rid of strange puck!")
            else:
                closest_puck = get_closest_puck(cluster_array_msg)
                if closest_puck == None or \
                    closest_puck.type != self.carried_type:
                    self.transition("DE_SCAN", "Closest p. lost/not right type")
                if get_puck_distance(closest_puck) < \
                                                self.CLUSTER_CONTACT_DISTANCE:
                    self.transition("DE_PUSH", "Contacted cluster --- Ok!")
                elif time_in_state > self.PLACEMENT_TIME:
                    self.transition("DE_SCAN", "Time out")

        elif self.state == "DE_PUSH":
            if time_in_state > self.PUSH_IN_TIME:
                self.transition("DE_BACKUP", "")
            
        elif self.state == "DE_BACKUP":
            self.target_type = None
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
        # Set gripper and velocity based on state.
        ######################################################################
        twist = None
        gripper_close = None
        if self.state == "PU_SCAN":
            self.activate_avoider(True)
            gripper_close = True

        elif self.state == "PU_TARGET":
            self.activate_avoider(False)
            closest_puck = get_closest_puck(cluster_array_msg)
            if closest_puck != None and closest_puck.type == self.target_type:
                twist = move_to_puck(closest_puck)
            else:
                # No puck of the right type in sight.  Assuming puck is out
                # of sight but almost within gripper.
                twist = forwards()
            gripper_close = False

        elif self.state == "DE_SCAN":
            self.activate_avoider(True)
            gripper_close = True

        elif self.state == "DE_TARGET":
            self.activate_avoider(False)
            closest_puck = get_closest_puck(cluster_array_msg)
            twist = move_to_puck(closest_puck)
            gripper_close = True

        elif self.state == "DE_PUSH":
            self.activate_avoider(False)
            twist = forwards()
            gripper_close = False

        elif self.state == "DE_BACKUP":
            self.activate_avoider(False)
            twist = backwards()
            gripper_close = False

        elif self.state == "DE_TURN":
            self.activate_avoider(False)
            twist = turn(self.random_turn_dir)
            gripper_close = True

        else:
            sys.exit("Unknown state")

        if twist != None:
            self.cmd_vel_publisher.publish(twist)
        if gripper_close != None:
            self.servo_publisher.publish(gripper_close)

    def shutdown_handler(self):
        # Stop the robot
        self.cmd_vel_publisher.publish(Twist())

if __name__ == '__main__':
    prob_seek = ProbSeek()
    rospy.spin()
