#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
import numpy as np
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from math import pi
from a_star_search import *

class State_L:
    def __init__(self):
        self.sub_poly = rospy.Subscriber('/putm/steering/poly_markers', MarkerArray, self.get_polynomial)
        self.polynomial = MarkerArray()
        self.epochs = 4
        self.step = 1
        self.angle_lattices = [-pi/3, -pi/4, -pi/6, 0, pi/6, pi/4, pi/3]
        self.start_point = (0, 0)
        self.end_point = self.start_point
        self.parent_nodes = [self.start_point]
        self.graph_connections = []
        self.heuristics = {}
        self.graph = Graph()
        self.path_points = [self.start_point]
        self.steering_path = []

    def get_polynomial(self, message):
        self.polynomial = message
        self.previous_points = self.path_points
        self.path_points = []

        for polynomial in self.polynomial.markers:
             self.path_points.append((polynomial.pose.position.x, polynomial.pose.position.y))
        self.path_points.sort(key = lambda x: x[0])

        if self.path_points == []:
            self.path_points = self.previous_points

        self.create_lattices()
        self.end_point = self.path_points[-1]
        self.add_endpoint_to_graph()
        self.create_graph_to_search()
        self.create_heuristics()
        self.search_graph()

    def create_lattices(self):
        while len(self.parent_nodes) < pow(len(self.angle_lattices), self.epochs):
            new_nodes = []
            for parent_node in self.parent_nodes:
                for angle in self.angle_lattices:
                    new_node_y = parent_node[1] + self.step * np.tan(angle)
                    new_node = (parent_node[0] + self.step, new_node_y)
                    new_nodes.append(new_node)

                    distance = self.calculate_distance(parent_node, new_node)
                    new_connection = (parent_node, new_node, distance)
                    self.graph_connections.append(new_connection)
            self.parent_nodes = new_nodes

    def calculate_distance(self, parent_node, new_node):
        distance = np.sqrt(pow(new_node[0] - parent_node[0], 2) + pow(new_node[1] - parent_node[1], 2))
        return distance

    def create_graph_to_search(self):
        for connection in self.graph_connections:
            self.graph.connect(str(connection[0]), str(connection[1]), connection[2])

    def add_endpoint_to_graph(self):
        for leaf in self.parent_nodes:
            distance = self.calculate_distance(leaf, self.end_point)
            self.graph_connections.append((leaf, self.end_point, distance))

    def create_heuristics(self):
        nodes = list(set([node[0] for node in self.graph_connections]))
        for node in nodes:
            current_point = self.path_points[-1]
            #for path_point in self.path_points[::-1]:
            #    if node[0] <= path_point[0]:
            #        current_point = path_point
            #    else:
            #        distance = self.calculate_distance(node, current_point)
            #        self.heuristics[str(node)] = distance
            distance = self.calculate_distance(node, current_point)
            self.heuristics[str(node)] = distance
            #print(distance)
        # add endpoint with value 0
        self.heuristics[str(self.end_point)] = 0

    def search_graph(self):
        self.steering_path = astar_search(self.graph, self.heuristics, str(self.start_point), str(self.end_point))
        print(self.steering_path)
        

if __name__ == '__main__':
    rospy.init_node('state_lattices')
    state_l = State_L()
    rospy.spin()





