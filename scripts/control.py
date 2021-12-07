import math

import numpy as np
import matplotlib.pyplot as plt
import csv
from numpy import linalg as LA


class PID():
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.e_prev = 0
        self.I = 0

    def update(self, ep):
        e = 2 - ep
        P = self.Kp * e
        self.I = self.I + self.Ki * e * 0.1
        D = self.Kd * (e - self.e_prev) / 0.1
        self.e_prev = e

        return 2 - (P + self.I + D)


def find_nearest_ponits(BicycleModel, last_index, waypoints):
    min = float('inf')
    index = 0
    bicycle_pos = [BicycleModel.x + (BicycleModel.Lf) * math.cos(BicycleModel.yaw),
                       BicycleModel.y + (BicycleModel.Lf) * math.sin(BicycleModel.yaw)]

    # bicycle_pos = [BicycleModel.rear_x,
    #                    BicycleModel.rear_]

    old_dis = distance(waypoints[last_index], bicycle_pos)
    new_dis = distance(waypoints[last_index + 1], bicycle_pos)

    print("old",old_dis , 'new', new_dis)
    print([waypoints[last_index+2], waypoints[last_index+1]])


    if new_dis < old_dis:
        return last_index + 1, [waypoints[last_index+2], waypoints[last_index+1]]
    else:
        return last_index, [waypoints[last_index+1], waypoints[last_index]]



    # for points in waypoints:
    #     dis = distance(points, bicycle_pos)
    #     if min > dis:
    #         min = dis
    #         nearest[0] = index
    #         if index == 0:
    #             nearest[1] = 1
    #         else:
    #             if distance(waypoints[index + 1], bicycle_pos) < distance(waypoints[index - 1], bicycle_pos):
    #                 nearest[1] = index + 1
    #             else:
    #                 nearest[1] = index - 1
    #     index = index + 1
    #
    #
    # near_points = [waypoints[nearest[0]], waypoints[nearest[1]]]
    # last_waypoint_index = nearest[0]
    # return near_points


def distance(pos1, pos2):
    dis = math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])
    return dis


def import_path(file_adress):
    with open(file_adress) as waypoints_file_handle:
        waypoints = list(csv.reader(waypoints_file_handle,
                                    delimiter=',',
                                    quoting=csv.QUOTE_NONNUMERIC))
        waypoints_np = np.array(waypoints)
    return waypoints



def steer_angle(BicycleModel, waypoints, last_index):
    # Calculate heading error
    data = []
    yaw_path = np.arctan2(abs(waypoints[1][1] - waypoints[0][1]), abs(waypoints[1][0] - waypoints[0][0]))
    yaw_diff = normalize_angle(yaw_path - BicycleModel.yaw)

    # Calculate crosstrack error
    k_e = 2.5
    k_v = 0

    p1 = np.asarray(waypoints[0])
    p2 = np.asarray(waypoints[1])
    p3 = np.asarray([BicycleModel.x + (BicycleModel.Lf) * math.cos(BicycleModel.yaw),
                     BicycleModel.y + (BicycleModel.Lf) * math.sin(BicycleModel.yaw)])

    cross_track_error = np.abs(np.cross(p2-p1, p1-p3)) / LA.norm(p2-p1)

    yaw_cross_track = np.arctan2(BicycleModel.y - waypoints[1][1], BicycleModel.x - waypoints[1][0])
    yaw_path2ct = yaw_path - yaw_cross_track

    if yaw_path2ct > np.pi:
        yaw_path2ct -= 2 * np.pi
    if yaw_path2ct < - np.pi:
        yaw_path2ct += 2 * np.pi

    if yaw_path2ct > 0:
        cross_track_error = abs(cross_track_error)
    else:
        cross_track_error = - abs(cross_track_error)
        

    pid_controller = PID(1, 0.05, 0.1)


    # yaw_diff_crosstrack = np.arctan(k_e * cross_track_error / (k_v + BicycleModel.vx))
    yaw_diff_crosstrack = np.arctan2(k_e * cross_track_error, k_v + BicycleModel.vx)
    steer = yaw_diff + yaw_diff_crosstrack
    steer_speed = pid_controller.update(BicycleModel.vx)

    # if last_index >= 5:
    #     steer = -steer

    # print('yaw_diff', yaw_diff, 'yaw_diff_crosstrack', yaw_diff_crosstrack,'BicycleModel.yaw', BicycleModel.yaw, last_index)
    return steer, steer_speed, cross_track_error

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

def pure_pursuit_steer_control(state, waypoints):


    alpha = math.atan2(waypoints[1][1] - state.rear_y, waypoints[1][0] - state.rear_x) - state.yaw
    delta = math.atan2(4.0 * state.L * math.sin(alpha) / state.Lf, 1.0)

    return delta