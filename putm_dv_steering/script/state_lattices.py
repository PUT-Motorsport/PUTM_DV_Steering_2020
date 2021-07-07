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
        self.pub_lattices = rospy.Publisher('/putm/steering/state_lattices_path', MarkerArray, queue_size=10)
        self.polynomial = MarkerArray()
        self.epochs = 4
        self.step = 1
        self.num_steps = 0
        self.angle_lattices = [-pi/3, -pi/4, -pi/6, 0, pi/6, pi/4, pi/3]
        self.start_point = (0, 0)
        self.end_point = self.start_point
        self.parent_nodes = [self.start_point]
        self.graph_connections = []
        self.heuristics = {}
        self.path_points = [self.start_point]
        self.steering_graph, self.steering_list = [], []

    def get_polynomial(self, message):
        self.polynomial = message
        self.previous_points = self.path_points
        self.path_points = []

        for polynomial in self.polynomial.markers:
             self.path_points.append((polynomial.pose.position.x, polynomial.pose.position.y))
        self.path_points.sort(key = lambda x: x[0])

        if self.path_points == []:
            self.path_points = self.previous_points

        # reset values for another graph connections
        self.graph = Graph()
        self.graph_connections = []
        self.heuristics = {}
        self.parent_nodes = [self.start_point]
        self.num_steps = 0
        self.end_point = self.path_points[-1]


        self.create_lattices()
        self.add_endpoint_to_graph()
        self.create_graph_to_search()

        self.heuristics = self.create_heuristics()
        self.search_graph()

        self.convert_graph_to_points()
        lattices_array = self.draw_lattices(self.steering_list)
        self.pub_lattices.publish(lattices_array)


    def create_lattices(self):
        while (len(self.parent_nodes) < pow(len(self.angle_lattices), self.epochs)) and (self.num_steps < self.end_point[0]-1):
            self.num_steps += self.step
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
        heuristic = {}
        nodes = list(set([node[0] for node in self.graph_connections]))
        for node in nodes:
            current_point = self.path_points[-1]
            for path_point in self.path_points[::-1]:
                if node[0] <= path_point[0]:
                    current_point = path_point
                else:
                    distance = self.calculate_distance(node, current_point)
                    heuristic[str(node)] = distance
        # add heuristic for the  endpoint
        heuristic[str(self.end_point)] = 0
        return heuristic

    def search_graph(self):
        self.steering_graph = astar_search(self.graph, self.heuristics, str(self.start_point), str(self.end_point))

    def convert_graph_to_points(self):
        self.steering_list = []
        for node in self.steering_graph:
            point = node.split(':')[0]
            self.steering_list.append(point)
        print(self.steering_list)

    def draw_lattices(self, points):
        markers_arr = MarkerArray()
        markers_arr.markers = []
        for index, point in enumerate(points):
                point = eval(point)
                marker = Marker()
                marker.type=Marker.SPHERE
                marker.id=index
                marker.lifetime=rospy.Duration(0.1)
                marker.pose=Pose(Point(point[0], point[1], 0), Quaternion(0, 0, 0, 1))
                marker.scale=Vector3(0.1, 0.1, 0.1)
                marker.header=Header(frame_id='fsds/cam1')
                marker.color=ColorRGBA(1.0, 0.0, 0.0, 0.8)
                markers_arr.markers.append(marker)
        return markers_arr

if __name__ == '__main__':
    rospy.init_node('state_lattices')
    state_l = State_L()
    rospy.spin()





