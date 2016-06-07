import math, random
from bupimo_msgs.msg import Puck
from bupimo_msgs.msg import PuckArray
from geometry_msgs.msg import Twist

from bupimo_utils.math_utils import sign
from bupimo_utils.pucks_and_clusters import get_closest_puck
from bupimo_utils.obstacles import get_closest_obstacle
from bupimo_utils.angles import constrain_angle

FORWARD_SPEED = 0.25
ROT_SPEED = 5.0

PI_OVER_2 = math.pi / 2.0

# This is the one little bit of internal state in this module which is used
# in 'wander_while_avoiding'.
last_wander_angular = 0

# List containing a count for which the index (of 'castobstacles') which has
# the miniumum rho ...
min_rho_accum = None

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
        twist.angular.z = 20.0 * puck.position.y

    return twist

def move_towards_bearing(bearing):
    """Generate a twist message that moves us towards the given puck."""
    twist = Twist()
    bearing = constrain_angle(bearing)
    twist.linear.x = FORWARD_SPEED
    twist.angular.z = ROT_SPEED * (bearing / math.pi)
    return twist


def wander_while_avoiding_castobs(castobstacle_array_msg):
    """Random walk while avoiding cast obstacles."""
    global min_rho_accum

    if castobstacle_array_msg == None:
        # The message has probably just not been published yet, go forwards
        return forwards()

    # First get the closest cast obstacle.  We will turn away from this if
    # it is too close.
    closest_obs = None
    closest_rho = float('inf')
    closest_index = None
    n = len(castobstacle_array_msg.obstacles):
    for i in range(n):
        obs = castobstacle_array_msg.obstacles[i]
        if obs.rho < closest_rho:
            closest_obs = obs
            closest_rho = obs.rho
            closest_index = i
    assert closest_obs != None
    
    # Accumulate the closest obstacle index
    if min_rho_accum == None:
        # Initialize with all zeros
        min_rho_accum = [0 for i in range(n)]

    min_rho_accum[closest_index] += 1
    gamma = 0.9
    for i in range(n):
        min_rho_accum[i] *= gamma

    # There are two options: if avoid is false then we seek the most open space.
    # If true then we avoid that obstacle.
    avoid = False
    if max(min_rho_accum) < 9:
        avoid = True
        
    if not avoid:
        # For each cast obstacle we will specify a vector whose length is given
        # by the 'rho' value.  We then sum all of these up and try and move in
        # this direction.
        vx = 0
        vy = 0
        for obs in castobstacle_array_msg.obstacles:
            vx = vx + obs.rho**2 * math.cos(obs.theta)
            vy = vy + obs.rho**2 * math.sin(obs.theta)
        bearing = math.atan2(vy, vx)
        move_towards_bearing(bearing)

    else:
        closest_theta = constrain_angle(closest_obs.theta)
        if closest_theta > 0 and closest_theta < PI_OVER_2:
            move_towards_bearing(closest_theta - PI_OVER_2)
        elif closest_theta < 0 and closest_theta > -PI_OVER_2:
            move_towards_bearing(closest_theta + PI_OVER_2)
        else:
            # The way is clear.  
            twist.linear.x = FORWARD_SPEED

    return twist
