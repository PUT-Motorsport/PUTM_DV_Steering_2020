import pygame
import matplotlib.pyplot as plt

from scripts import model
from scripts import grafics
from scripts import control


WAYPOINTS_FILENAME = 'paths/file2.txt'
last_waypoint_index = 0
done = False

ratio = 20  # Ratio between calculated model postion and display model postion, calculated model ostion [x,y] display model postion [x*ratio,y*ratio]

screen = pygame.display.set_mode([1600, 800])
pygame.display.set_caption("PUT Motorsport Driverles Model")
clock = pygame.time.Clock()
pygame.font.init()
pygame.init()


B_Model = model.BicycleModel()
waypoints = control.import_path(WAYPOINTS_FILENAME)
last_index = 0
angle_to_steer_list = []

data = []
if __name__ == '__main__':
    while not done:

        clock.tick(80)
        screen.fill((0, 0, 0))
        pressed = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                print(pos)

        last_index, near_points = control.find_nearest_ponits(B_Model, last_index, waypoints)
        print(last_index)
        angle_to_steer, steer_speed, cross_track_error = control.steer_angle(B_Model, near_points, last_index)
        angle_to_steer = control.pure_pursuit_steer_control(B_Model, near_points)
        angle_to_steer_list.append(angle_to_steer)

        # print(angle_to_steer)

        B_Model.update(steer_speed, angle_to_steer)
        grafics.show_model(B_Model, near_points,cross_track_error, screen, ratio)
        grafics.show_path(waypoints, screen, ratio)

        pygame.display.flip()
        # time.sleep(0.1)



    plt.show()
