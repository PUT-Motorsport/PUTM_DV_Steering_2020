import math

import pygame
import matplotlib.pyplot as plt
import pygame.freetype
from numpy import linalg as LA


BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 200, 0)
RED = (255, 0, 0)


def show_model(BicycleModel, near_points,cross_track_error, screen, ratio):
    # pygame.draw.circle(screen,BLUE,(int(BicycleModel.x * ratio), int(BicycleModel.y * ratio)), int(abs(cross_track_error) * ratio))
    print(near_points)
    pygame.draw.line(screen, WHITE, (BicycleModel.x* ratio, BicycleModel.y* ratio), (near_points[0][0]* ratio, near_points[0][1]* ratio))
    pygame.draw.line(screen, RED, (BicycleModel.x* ratio, BicycleModel.y* ratio), (near_points[1][0]* ratio, near_points[1][1]* ratio))

    pygame.draw.polygon(screen, GREEN,
                        rect(BicycleModel.x, BicycleModel.y, BicycleModel.yaw, BicycleModel.L * ratio, 0.4 * ratio, ratio))

    pygame.draw.polygon(screen, RED, rect(BicycleModel.x + (BicycleModel.Lf) * math.cos(BicycleModel.yaw),
                                          BicycleModel.y + (BicycleModel.Lf) * math.sin(BicycleModel.yaw),
                                          BicycleModel.delt + BicycleModel.yaw, 1 * ratio, 0.5 * ratio, ratio))

    pygame.draw.polygon(screen, RED, rect(BicycleModel.x - (BicycleModel.Lr) * math.cos(BicycleModel.yaw),
                                          BicycleModel.y - (BicycleModel.Lr) * math.sin(BicycleModel.yaw),
                                          BicycleModel.yaw, 1 * ratio, 0.5 * ratio, ratio))

    pygame.draw.polygon(screen, BLUE, rect(BicycleModel.x, BicycleModel.y, 0.0, 0.4 * ratio, 0.4 * ratio, ratio))


def rect(x, y, angle, w, h, ratio):
    return [translate(x * ratio, y * ratio, angle, -w / 2, h / 2),
            translate(x * ratio, y * ratio, angle, w / 2, h / 2),
            translate(x * ratio, y * ratio, angle, w / 2, -h / 2),
            translate(x * ratio, y * ratio, angle, -w / 2, -h / 2)]


def translate(x, y, angle, px, py):
    x1 = x + px * math.cos(angle) - py * math.sin(angle)
    y1 = y + px * math.sin(angle) + py * math.cos(angle)
    if x1 == 0.0:
        x1 = int(0)
    if y1 == 0.0:
        y1 = int(0)
    return [(x1), (y1)]



def round_up(n, decimals=0):
    multiplier = 10 ** decimals
    return math.ceil(n * multiplier) / multiplier


def show_path(waypoints, screen, ratio):
    for point in waypoints:
        pygame.draw.polygon(screen, GREEN, rect(point[0], point[1], 0, 2, 2, ratio))

nearest = [1, 1]
