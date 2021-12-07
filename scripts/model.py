import math
import numpy as np
import pygame
import pygame.freetype
from numpy import linalg as LA


MAX_STEER = np.radians(60.0)  # [rad]
MAX_VELOCITY = 10.0  # [m/s]
MIN_VELOCITY = 0.4 # [m/s]

L = 2.5 # [m]
dt = 0.1
Lr = L / 2.0  # [m]
Lf = L - Lr
Cf = 1600.0 * 2.0  # Tire stiffness N/slip

Cr = 1700.0 * 2.0  # Tire stiffness N/slip
Iz = 2200.0  # kg/m2
m = 1000.0  # kg


class BicycleModel():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=MIN_VELOCITY, vy=0, omega=0, delt=0):
        # Model position
        self.x = x 
        self.y = y
        self.vx = vx
        self.vy = vy
        # Model parameters
        self.m = m
        self.Iz = Iz
        self.Cr = Cr
        self.Cf = Cf     
        self.L = L
        self.Lf = Lf
        self.Lr = Lr
        # Angles init
        self.yaw = yaw
        self.omega = omega
        self.delt = delt
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01

        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))



    def update(self, throttle, delta):
        """
        Calculates model positon, velocity ... based on non-linear bicycle model 
        :param throttle: (float) [m/s2]
        :param delta: (float) [rad] 
        :return: none
        """
        ############### FOR TEASTING ###########

        throttle = 0 

        # Equations used to calculate state of the model
        delta = np.clip(delta, -MAX_STEER, MAX_STEER)
        self.delt = delta
        self.x = self.x + self.vx * math.cos(self.yaw) * dt - self.vy * math.sin(self.yaw) * dt
        self.y = self.y + self.vx * math.sin(self.yaw) * dt + self.vy * math.cos(self.yaw) * dt
        self.yaw = self.yaw + self.omega * dt
        self.yaw = normalize_angle(self.yaw)
        Ffy = -Cf * math.atan2(((self.vy + Lf * self.omega) / self.vx - delta), 1.0)
        Fry = -Cr * math.atan2((self.vy - Lr * self.omega) / self.vx, 1.0)
        R_x = self.c_r1 * self.vx
        F_aero = self.c_a * self.vx ** 2
        F_load = F_aero + R_x
        self.vx = self.vx + (throttle - Ffy * math.sin(delta) / m - F_load/m + self.vy * self.omega) * dt
        velocity = np.clip(self.vx, 0.4, MAX_VELOCITY)
        self.vx = 2.0
        self.vy = self.vy + (Fry / m + Ffy * math.cos(delta) / m - self.vx * self.omega) * dt
        self.omega = self.omega + (Ffy * Lf * math.cos(delta) - Fry * Lr) / Iz * dt
        self.rear_x = self.x - ((self.L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.L / 2) * math.sin(self.yaw))
        return [self.x, self.y]

    
    def update_kinematic(self, a, delta):
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.vx / self.L * math.tan(delta) * dt
        self.vx = 1.0
        self.rear_x = self.x - ((self.L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.L / 2) * math.sin(self.yaw))


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle    






