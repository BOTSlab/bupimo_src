import math

def get_closest_obstacle(obstacle_array_msg):
    closest_obs = None
    closest_dist = float('inf')
    for obs in obstacle_array_msg.obstacles:
        dist = math.sqrt(obs.position.x**2 + obs.position.y**2)
        if dist < closest_dist:
            closest_obs = obs
            closest_dist = dist
    return closest_obs

