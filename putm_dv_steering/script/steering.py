#!/usr/bin/env python3

import math
import yaml
import numpy as np
import rospy as rp

from geometry_msgs.msg import TwistStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from fs_msgs.msg import ControlCommand, FinishedSignal, GoSignal


class AutonomousSteering:
    def __init__(self):
        self.max_throttle = rp.get_param('/vehicle/max_throttle')
        self.target_speed = rp.get_param('/vehicle/target_speed')
        self.max_steering = rp.get_param('/vehicle/max_steering')

        rp.Subscriber("/fsds/gss", TwistStamped, self.gss_signal_callback)
        rp.Subscriber('/putm/steering/state_lattices_path', MarkerArray, self.get_polynomial)

        self.go_publisher = rp.Publisher('/putm/steering/signal/go', GoSignal, queue_size=1)
        self.control_publisher = rp.Publisher('/putm/steering/control_command', ControlCommand, queue_size=10)
        self.finished_publisher = rp.Publisher('/putm/steering/finished', FinishedSignal, queue_size=1)

        self.steering_time = 0
        self.velocity = 0.0
        self.brake = 1 # float64 0..1
        self.go = False
        self.steering = 0.0
        self.path_points = []
        self.blue_cones = []
        self.yellow_cones = []
        self.steering_points = []

    def get_polynomial(self, message):
        self.polynomial = message
        self.previous_points = self.steering_points
        self.steering_points = []

        for polynomial in self.polynomial.markers:
             self.steering_points.append((polynomial.pose.position.x, polynomial.pose.position.y))
        self.steering_points.sort(key = lambda x: x[0])

        if self.steering_points == []:
            self.steering_points = self.previous_points

        try:
            self.steering = self.calculate_steering()
        except:
            print("Cannot calculate steering!")

    def start(self, *args):
        gs = GoSignal()
        gs.mission = 'Trackdrive'
        self.go_publisher.publish(gs)
        self.go = True
        self.brake = 0

    def gss_signal_callback(self, data):
        v_x = data.twist.linear.x
        v_y = data.twist.linear.y

        # Calculate vehicle velocity
        self.velocity = math.sqrt(math.pow(v_x, 2) + math.pow(v_y, 2))

    def calculate_steering(self):
        steering = 0.0
        print(self.steering_points)

        if steering > self.max_steering:
            steering = self.max_steering
        elif steering < -self.max_steering:
            steering = -self.max_steering
        if self.brake == 1:
            steering = 0.0
        elif abs(steering) > self.max_steering / 4 * 3:
            self.brake = abs(steering) / 2
        else:
            self.brake = 0
        return steering

    def calculate_throttle(self):
        throttle = self.max_throttle * max(1 - self.velocity / self.target_speed, 0)
        if self.brake == 1:
            throttle = 0
        return throttle

    def publish_control_command(self, throttle, steering):
        if self.go:
            cc = ControlCommand()
            cc.header.stamp = rp.Time.now()
            cc.throttle = throttle
            cc.steering = steering
            cc.brake = self.brake
            self.control_publisher.publish(cc)

    def emergency_brake(self):
        self.publish_finish_signal()

    def publish_finish_signal(self, *args):
        self.brake = 1
        while self.velocity > 0:
            self.publish_control_command(throttle=0, steering=0)

        self.finished_publisher.publish(FinishedSignal())

    def go_autonomous(self, *args):
        self.publish_control_command(throttle=self.calculate_throttle(), steering=self.steering)

if __name__ == '__main__':
    rp.init_node('steering', log_level=rp.DEBUG)

    AS = AutonomousSteering()
    rp.Timer(rp.Duration(5.0), AS.start, oneshot=True)
    # rp.Timer(rp.Duration(0.1), AS.go_autonomous)

    while not rp.is_shutdown():
        rp.spin()
