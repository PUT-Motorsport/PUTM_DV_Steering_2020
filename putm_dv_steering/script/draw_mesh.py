#!/usr/bin/env python
from scipy.spatial import Delaunay
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
import numpy as np
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class Drawer:
    def __init__(self):

        self.sub_blue = rospy.Subscriber("/putm/vision/blue_cones_position", PoseArray, self.blue_callback)
        self.sub_yellow = rospy.Subscriber("/putm/vision/yellow_cones_position", PoseArray, self.yellow_callback)

        self.pub_blue = rospy.Publisher('/putm/steering/blue_markers', MarkerArray, queue_size=10)
        self.pub_yellow = rospy.Publisher('/putm/steering/yellow_markers', MarkerArray, queue_size=10)
        self.pub_triangles = rospy.Publisher('/putm/steering/triangles_markers', MarkerArray, queue_size=10)
        self.pub_path = rospy.Publisher('/putm/steering/path_markers', MarkerArray, queue_size=10)
        self.pub_poly = rospy.Publisher('/putm/steering/poly_markers', MarkerArray, queue_size=10)

        self.camera_range = rospy.get_param('/sensors/cones_range_cutoff')
        self.blue_cones, self.yellow_cones = PoseArray(), PoseArray()
        self.blue_list, self.yellow_list, self.points_list, self.path_points, self.poly_points = [], [], [], [], []


    def blue_callback(self, message):
        self.blue_cones = message

    def yellow_callback(self, message):
        self.yellow_cones = message

        self.blue_list = self.poses_2_list(self.blue_cones.poses)
        self.yellow_list = self.poses_2_list(self.yellow_cones.poses)
        self.points_list = self.blue_list + self.yellow_list

        self.points_list = self.filter_points(self.points_list)

        blue_cones_arr = self.draw_cones(self.blue_cones, "blue")
        yellow_cones_arr = self.draw_cones(self.yellow_cones, "yellow")
        self.pub_blue.publish(blue_cones_arr)
        self.pub_yellow.publish(yellow_cones_arr)

        try:
            triangles = self.make_triangulation()
            self.path_points = self.create_path_points(self.points_list, triangles)
            self.poly_points = self.fit_poly(self.path_points)

            triangle_markers = self.draw_triangles(triangles)
            path_arr = self.draw_path(self.path_points)
            poly_arr = self.draw_path(self.poly_points)

            self.pub_triangles.publish(triangle_markers)
            self.pub_path.publish(path_arr)
            self.pub_poly.publish(poly_arr)

        except:
            print("Not enough points to build a mesh!")

    def poses_2_list(self, points):
        new_list = []
        for point in points:
            new_list.append([point.position.x, point.position.y])
        return new_list

    def filter_points(self, points):
        new_points = []
        for point in points:
            distance = np.sqrt(point[0]**2 + point[1]**2)
            # Filter out points that may be disturbing in the car's nearest area.
            if (distance < 0.2 * self.camera_range and np.abs(point[1]) > 0.4) or (0.2 * self.camera_range < distance < self.camera_range):
                new_points.append(point)
        return new_points

    def make_triangulation(self):
        points_array = np.asarray(self.points_list)
        tri = Delaunay(points_array)
        return tri.simplices

    def calculate_mid_point(self, pointA, pointB):
        x = (pointA[0] + pointB[0]) / 2
        y = (pointA[1] + pointB[1]) / 2
        return [x,y]

    def create_path_points(self, points, triangles):
        path_points = []
        for triangle in triangles:
            # Not optimal at this moment.
            main_vertexes = [self.points_list[vertex] for vertex in triangle]
            sub_vertexes = main_vertexes.copy()
            for vertex1 in main_vertexes:
               for vertex2 in sub_vertexes:
                    if (vertex1 in self.blue_list and vertex2 in self.yellow_list and vertex1[1]>vertex2[1]) or (vertex1 in self.yellow_list and vertex2 in self.blue_list and vertex2[1]>vertex1[1]):
                        mid_point = self.calculate_mid_point(vertex1, vertex2)
                        if mid_point not in path_points:
                            path_points.append(mid_point)
        return path_points

    def fit_poly(self, points):
        new_points = []
        if points != []:
            x_values = np.array(points)[:,0]
            x_values = np.append(x_values, 0)
            y_values = np.array(points)[:,1]
            y_values = np.append(y_values, 0)
            coeffs = np.polyfit(x_values, y_values, 2)
            poly = np.poly1d(coeffs)
            new_y_values = [poly(x) for x in x_values]
            new_points = np.array(list(zip(x_values, new_y_values)))
        return new_points

    def draw_cones(self, cones, color):
        markers_arr = MarkerArray()
        markers_arr.markers = []
        for index, points in enumerate(cones.poses):
                marker = Marker()
                marker.type=Marker.SPHERE
                marker.id=index
                marker.lifetime=rospy.Duration(0.1)
                marker.pose=Pose(Point(points.position.x, points.position.y, 0), Quaternion(0, 0, 0, 1))
                marker.scale=Vector3(0.1, 0.1, 0.1)
                marker.header=Header(frame_id='fsds/cam1')
                if color == "blue":
                    marker.color=ColorRGBA(0.0, 0.0, 1.0, 0.8)
                else:
                    marker.color=ColorRGBA(1.0, 1.0, 0.0, 0.8)
                markers_arr.markers.append(marker)
        return markers_arr

    def draw_triangles(self, triangles):
        markers_tri = MarkerArray()
        markers_tri.markers = []
        for index, triangle in enumerate(triangles):
                marker = Marker()
                marker.type=Marker.LINE_LIST
                marker.id=index
                marker.lifetime=rospy.Duration(0.001)
                marker.scale=Vector3(0.05, 0.0, 0.0)
                marker.points = [Point(self.points_list[triangle[0]][0], self.points_list[triangle[0]][1], 0),
                                 Point(self.points_list[triangle[1]][0], self.points_list[triangle[1]][1], 0),
                                 Point(self.points_list[triangle[1]][0], self.points_list[triangle[1]][1], 0),
                                 Point(self.points_list[triangle[2]][0], self.points_list[triangle[2]][1], 0),
                                 Point(self.points_list[triangle[2]][0], self.points_list[triangle[2]][1], 0),
                                 Point(self.points_list[triangle[0]][0], self.points_list[triangle[0]][1], 0)]
                marker.header=Header(frame_id='fsds/cam1')
                marker.color=ColorRGBA(0.0, 0.0, 1.0, 0.8)
                markers_tri.markers.append(marker)
        return markers_tri

    def draw_path(self, path_points):
        markers_arr = MarkerArray()
        markers_arr.markers = []
        for index, points in enumerate(path_points):
                marker = Marker()
                marker.type=Marker.SPHERE
                marker.id=index
                marker.lifetime=rospy.Duration(0.1)
                marker.pose=Pose(Point(points[0], points[1], 0), Quaternion(0, 0, 0, 1))
                marker.scale=Vector3(0.1, 0.1, 0.1)
                marker.header=Header(frame_id='fsds/cam1')
                marker.color=ColorRGBA(0.0, 1.0, 0.0, 0.8)
                markers_arr.markers.append(marker)
        return markers_arr

if __name__ == '__main__':
    rospy.init_node('draw_mesh')
    drawer = Drawer()
    rospy.spin()





