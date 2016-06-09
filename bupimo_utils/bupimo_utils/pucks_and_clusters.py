"""Useful functions for extracting information from PuckArray, Cluster, and
ClusterArray messages."""

import math

def get_puck_distance(puck):
    return math.sqrt(puck.position.x**2 + puck.position.y**2)

def get_closest_puck_in_cluster(cluster_msg):
    closest_puck = None
    closest_dist = float('inf')
    for puck in cluster_msg.array.pucks:
        dist = math.sqrt(puck.position.x**2 + puck.position.y**2)
        if dist < closest_dist:
            closest_puck = puck
            closest_dist = dist
    return closest_puck

def get_closest_puck(cluster_array_msg):
    closest_puck = None
    closest_dist = float('inf')
    for cluster in cluster_array_msg.clusters:
        for puck in cluster.array.pucks:
            dist = math.sqrt(puck.position.x**2 + puck.position.y**2)
            if dist < closest_dist:
                closest_puck = puck
                closest_dist = dist
    return closest_puck

def get_closest_puck_of_type(cluster_array_msg, puck_type):
    closest_puck = None
    closest_dist = float('inf')
    for cluster in cluster_array_msg.clusters:
        for puck in cluster.array.pucks:
            if puck.type == puck_type:
                dist = math.sqrt(puck.position.x**2 + puck.position.y**2)
                if dist < closest_dist:
                    closest_puck = puck
                    closest_dist = dist
    return closest_puck

def get_closest_puck_to_puck(cluster_array_msg, ref_puck, max_distance):
    """Return the closest puck to the given puck within max_distance."""
    closest_puck = None
    closest_dist = float('inf')
    for cluster in cluster_array_msg.clusters:
        for puck in cluster.array.pucks:
            dx = ref_puck.position.x - puck.position.x
            dy = ref_puck.position.y - puck.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < max_distance and dist < closest_dist:
                closest_puck = puck
                closest_dist = dist
    return closest_puck

def get_closest_puck_distance(cluster_array_msg):
    closest_dist = float('inf')
    for cluster in cluster_array_msg.clusters:
        for puck in cluster.array.pucks:
            dist = math.sqrt(puck.position.x**2 + puck.position.y**2)
            if dist < closest_dist:
                closest_dist = dist
    return closest_dist

def get_smallest_cluster(cluster_array_msg):
    smallest_cluster = None
    smallest_n = float('inf')
    for cluster in cluster_array_msg.clusters:
        n = len(cluster.array.pucks)
        if n < smallest_n:
            smallest_cluster = cluster
            smallest_n = n
    return smallest_cluster

def get_closest_cluster(cluster_array_msg):
    """The closest cluster is the one that contains the closest puck."""
    closest_cluster = None
    closest_dist = float('inf')
    for cluster in cluster_array_msg.clusters:
        for puck in cluster.array.pucks:
            dist = math.sqrt(puck.position.x**2 + puck.position.y**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_cluster = cluster
    return closest_cluster

def get_largest_cluster_of_type(cluster_array_msg, puck_type):
    largest_cluster = None
    largest_n = 0
    for cluster in cluster_array_msg.clusters:
        if cluster.type == puck_type:
            n = len(cluster.array.pucks)
            if n > largest_n:
                largest_cluster = cluster
                largest_n = n
    return largest_cluster
