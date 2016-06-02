#!/usr/bin/env python
"""
ROS Node which subscribes to 'pucks' and publishes 'clusters'.

The (x, y) coordinates of detected pucks are used to define nodes.  Edges are created between pairs of nodes of the same 'type' with a Euclidean distance less than the parameter 'cluster_distance_threshold'.  Clusters consist of a type, centroid, and an array of pucks  Note that clusters are homogeneous by definition, meaning that all pucks belonging to the same cluster share the same 'type'.

Andrew Vardy
"""

import sys
import rospy
import networkx as nx
import math
import tf
from bupimo_msgs.msg import Puck
from bupimo_msgs.msg import PuckArray
from bupimo_msgs.msg import Cluster
from bupimo_msgs.msg import ClusterArray
#from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

def pucks_callback(puck_array_msg):
    clusters = ClusterArray()
    clusters.header.frame_id = "/base_link"
    #clusters.header.stamp = rospy.Time(0)

    # Create lists of pucks of each type
    pucks_by_type = [[] for type in range(number_puck_obs_types + 1)]
    
    for type in range(number_puck_obs_types + 1):
        for puck in puck_array_msg.pucks:
            if puck.type == type:
                pucks_by_type[type].append(puck)
    for type in range(number_puck_obs_types + 1):
        pucks_of_type = pucks_by_type[type]
        #print "pucks_of_type: {0}".format(pucks_of_type)

        # Create the graph of all pucks of this type.  We will then decompose
        # this graph (if it isn't empty) into its connected components, which
        # are the clusters.
        G = nx.Graph()
        G.add_nodes_from(pucks_of_type)
        edges = []
        #print "pucks_in_graph: {0}".format(G.nodes())
        for node in G.nodes():
            for neighbour in G.nodes():
                dx = node.position.x - neighbour.position.x
                dy = node.position.y - neighbour.position.y
                distance = math.sqrt(dx*dx + dy*dy)
                if distance < cluster_distance_threshold:
                    G.add_edge(node, neighbour)

        #print"{0}".format(G.nodes())
        cluster_graphs = nx.connected_component_subgraphs(G)
        #print"{0}".format(subgraphs[0].nodes())
        
        for cluster_graph in cluster_graphs:
            cluster = Cluster()
            cluster.type = type
            cluster.position = centroid_of_graph(cluster_graph)
            cluster.array.pucks = cluster_graph.nodes()
            cluster.array.header.stamp = rospy.Time.now()
            cluster.array.header.frame_id = "/base_link"
            clusters.clusters.append(cluster)

    clusters_publisher.publish(clusters)

def centroid_of_graph(cluster_graph):
    sum_x = 0
    sum_y = 0
    for puck in cluster_graph:
        sum_x = sum_x + puck.position.x
        sum_y = sum_y + puck.position.y

    n = len(cluster_graph)
    return Point32(float(sum_x)/n, float(sum_y)/n, 0)
    

if __name__ == '__main__':
    global cluster_distance_threshold, number_puck_types, number_puck_obs_types, clusters_publisher, vis_centroids_publisher
    
    rospy.init_node('pucks_to_clusters')

    # Read the following parameters managed by the ROS parameter server:
    #   cluster_distance_threshold:  Distance between pucks used to define
    #                                clusters.
    #   number_puck_types:           (see blocks_to_pucks.py)
    #
    cluster_distance_threshold = rospy.get_param('cluster_distance_threshold')
    number_puck_types = rospy.get_param('number_puck_types')

    # We include "obstacle" pucks as a special type of puck.
    number_puck_obs_types = number_puck_types + 1

    # Publish to /clusters
    clusters_publisher = rospy.Publisher('/clusters', ClusterArray, queue_size=1)
    
    # Publish to /vis_centroids
    vis_centroids_publisher = rospy.Publisher('/vis_centroids', PointCloud, queue_size=1)

    # Subscribe to /pucks
    rospy.Subscriber('/pucks', PuckArray, pucks_callback)
    
    rospy.spin()
