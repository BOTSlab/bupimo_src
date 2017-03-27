import math, random
from bupimo_msgs.msg import Puck
from bupimo_msgs.msg import PuckArray
from geometry_msgs.msg import Twist

from bupimo_utils.math_utils import sign
from bupimo_utils.pucks_and_clusters import get_closest_puck
from bupimo_utils.obstacles import get_closest_obstacle
from bupimo_utils.angles import constrain_angle_neg_pos_pi

FORWARD_SPEED = 0.25
ROT_SPEED = 5.0

PI_OVER_2 = math.pi / 2.0

# This is the one little bit of internal state in this module which is used
# in 'wander_while_avoiding'.
last_wander_angular = 0

# Dictionary containing a count for the forward 'theta' values of castobstacles
min_rho_accum = {}
wander_state = "AVOID"
time_in_random = 0

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

def curve(direction):
    """Generate a twist message that moves us forward while turning in the given direction."""
    twist = Twist()
    twist.linear.x = FORWARD_SPEED
    twist.angular.z = direction * 0.25 * ROT_SPEED
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

def wander_while_avoiding_obs_pucks(obstacle_array_msg, cluster_array_msg):
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
        twist.angular.z = 10.0 * puck.position.y

    return twist

def move_forwards_to_bearing(bearing):
    """Generate a twist message that moves us towards the given bearing, always travelling forwards."""
    goalRy = math.sin(bearing)
    twist = Twist()
    twist.linear.x = FORWARD_SPEED
    twist.angular.z = ROT_SPEED * goalRy
    return twist

def move_towards_bearing(bearing):
    """Generate a twist message that moves us towards the given bearing."""
    goalRx = math.cos(bearing)
    goalRy = math.sin(bearing)
    twist = Twist()
    twist.linear.x = FORWARD_SPEED * goalRx
    twist.angular.z = ROT_SPEED * goalRy
    return twist


#
#def move_towards_bearing_aggressive(bearing):
#    """Generate a twist message that moves us towards the given bearing,
#including potentially going backwards."""
#    twist = Twist()
#    bearing = constrain_angle_neg_pos_pi(bearing)
#    if bearing > 0:
#        twist.linear.x = (PI_OVER_2 - bearing) / PI_OVER_2 * FORWARD_SPEED
#    else:
#        twist.linear.x = (bearing + PI_OVER_2) / PI_OVER_2 * FORWARD_SPEED
#    twist.angular.z = ROT_SPEED * (bearing / math.pi)
#    return twist
#

def wander_while_avoiding_castobs(castobstacle_array_msg):
    """Random walk while avoiding cast obstacles."""
    global min_rho_accum, wander_state, time_in_random

    if castobstacle_array_msg == None:
        # The message has probably just not been published yet, go forwards
        return forwards()

    # First get the closest cast obstacle in the front.
    closest_obs = None
    closest_rho = float('inf')
    for obs in castobstacle_array_msg.obstacles:
        if obs.theta < PI_OVER_2 and obs.theta > -PI_OVER_2 \
            and obs.rho < closest_rho:
            closest_obs = obs
            closest_rho = obs.rho
    assert closest_obs != None
    
    if len(min_rho_accum) == 0:
        # Initialize accumulator array
        for obs in castobstacle_array_msg.obstacles:
            # Include only front directions
            if obs.theta < PI_OVER_2 and obs.theta > -PI_OVER_2:
                min_rho_accum[obs.theta] = 0

    # Print accumulator
    #print(min_rho_accum)

    if wander_state == "AVOID":
        # Add to accumulator
        for obs in castobstacle_array_msg.obstacles:
            if obs == closest_obs:
                min_rho_accum[obs.theta] += 1

    else: # RANDOM state
        time_in_random += 1

    # State transition
    if wander_state == "AVOID" and max(min_rho_accum.values()) == 20:
        wander_state = "RANDOM"
        time_in_random = 0
    elif wander_state == "RANDOM" and time_in_random == 5:
        wander_state = "AVOID"
        # Zero accumulator dictionary
        for obs in castobstacle_array_msg.obstacles:
            min_rho_accum[obs.theta] = 0

    # Behaviour for state
    if wander_state == "AVOID":
        #print("AVOID")
        closest_theta = constrain_angle_neg_pos_pi(closest_obs.theta)
        if closest_rho > 110 or math.fabs(closest_theta) > PI_OVER_2:
            # The way is clear.  
            return forwards()
        elif closest_theta > 0:
            return move_towards_bearing(closest_theta - PI_OVER_2)
        else:
            return move_towards_bearing(closest_theta + PI_OVER_2)

    elif wander_state == "RANDOM":
        #print("RANDOM")
        r = random.randint(0, 6)
        if r <= 3:
            return backwards()
        elif r == 4:
            return forwards()
        elif r == 5:
            return turn(-1)
        else:
            return turn(1)
