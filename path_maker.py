import pygame
import math


def rect(x, y, angle, w, h):
    return [translate(x, y, angle, -w / 2, h / 2),
            translate(x, y, angle, w / 2, h / 2),
            translate(x, y, angle, w / 2, -h / 2),
            translate(x, y, angle, -w / 2, -h / 2)]


def translate(x, y, angle, px, py):
    x1 = x + px * math.cos(angle) - py * math.sin(angle)
    y1 = y + px * math.sin(angle) + py * math.cos(angle)
    return [int(x1), int(y1)]


GREEN = (0, 200, 0)
screen = pygame.display.set_mode([1600, 800])
pygame.init()
done = False
list_pos = []

while not done:  # your main loop
    # get all events
    ev = pygame.event.get()
    for point in list_pos:
        pygame.draw.polygon(screen, GREEN, rect(point[0], point[1], 0, 2, 2))

    # proceed events
    for event in ev:
        # handle QUIT
        if event.type == pygame.QUIT:
            done = True
        # handle MOUSEBUTTONUP
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            if pos not in list_pos:
                list_pos.append(list(pos))

    pygame.display.flip()

print((list_pos))

for index in range(len(list_pos)):
    print(index)
    list_pos[index][0] = list_pos[index][0] / 20
    list_pos[index][1] = list_pos[index][1] / 20

with open("paths/file3.txt", "w") as output:
    for points in list_pos:
        output.write(str(points[0]))
        output.write(', ',)
        output.write(str(points[1]))
        output.write('\n')


# with open("file.txt", "r") as f:
#     for item in f:
#         points = item
#
# for point in points:
#     print(point)
