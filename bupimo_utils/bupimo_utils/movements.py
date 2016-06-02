import math, random
from bupimo_msgs.msg import Puck
from bupimo_msgs.msg import PuckArray
from geometry_msgs.msg import Twist

from bupimo_utils.math_utils import sign
from bupimo_utils.pucks_and_clusters import get_closest_puck
from bupimo_utils.obstacles import get_closest_obstacle

FORWARD_SPEED = 0.25
ROT_SPEED = 5.0

# This is the one little bit of internal state in this module which is used
# in 'wander_while_avoiding'.
last_wander_angular = 0

def forwards():
    twist = Twist()
    twist.linear.x = FORWARD_SPEED
    return twist

def backwards():
    twist = Twist()
    twist.linear.x = -FORWARD_SPEED
    return twist

def turn(direction):
    """Turn at a constant rate left (direction==1) or right (direction==-1)"""
    twist = Twist()
    twist.angular.z = direction * ROT_SPEED
    return twist

def wander_while_avoiding(obs_or_puck):
    global last_wander_angular

    twist = Twist()
    twist.linear.x = FORWARD_SPEED
    if obs_or_puck != None and math.fabs(obs_or_puck.position.y) < 0.10:
        # Avoid this obstacle puck
        twist.angular.z = -ROT_SPEED * sign(obs_or_puck.position.y)
        last_wander_angular = 0
    else:
        # The way is clear.  Choose a random direction by adding a small random
        # component to the angular value from last time.
        twist.angular.z = last_wander_angular \
                          + 2.0 * (random.random() - 0.5)
        last_wander_angular = twist.angular.z
    return twist

def wander_while_avoiding_obs_pucks(obstacle_array_msg, cluster_array_msg, \
                                    last_twist):
    """Random walk while avoiding both obstacles and pucks.  If either or both
    arguments are None then they will be ignored."""

    if obstacle_array_msg == None:
        obs = None
    else:
        obs = get_closest_obstacle(obstacle_array_msg)
    if cluster_array_msg == None:
        puck = None
    else:
        puck = get_closest_puck(cluster_array_msg)

    if obs == None and puck == None:
        return wander_while_avoiding(None)
    elif obs != None and puck == None:
        return wander_while_avoiding(obs)
    elif obs == None and puck != None:
        return wander_while_avoiding(puck)
    else:
        obs_dist = math.sqrt(obs.position.x**2 + obs.position.y**2)
        puck_dist = math.sqrt(puck.position.x**2 + puck.position.y**2)
        if obs_dist < puck_dist:
            return wander_while_avoiding(obs)
        else:
            return wander_while_avoiding(puck)

def move_to_puck(puck):
    """Generate a twist message that moves us towards the given puck."""
    twist = Twist()
    if puck != None:
        twist.linear.x = FORWARD_SPEED #0.5# + 1.0 * puck.position.x
        twist.angular.z = 20.0 * puck.position.y

    return twist
