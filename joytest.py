import pygame
from pygame.locals import *
import os
import time

os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.joystick.init()
DEVICE = "Logitech Gamepad F710"

f710 = None
for i in range(0, pygame.joystick.get_count()):
    if pygame.joystick.Joystick(i).get_name() == DEVICE:
        f710 = pygame.joystick.Joystick(i)
if not f710:
    print("Not found %s" % DEVICE)
    exit(0)

f710.init()
num_buttons = f710.get_numbuttons()
num_axes = f710.get_numaxes()
num_hats = f710.get_numhats()
print("device name: %s" % f710.get_name())
print("num of button: %d" % num_buttons)
print("num of axis: %d" % num_axes)
print("num of hat: %d" % num_hats)

pygame.init()
axes = [0]*num_axes
buttons = [0]*num_buttons
hats_x = [0]*num_hats
hats_y = [0]*num_hats

while True:
    for e in pygame.event.get():
        if e.type == pygame.locals.JOYAXISMOTION:  # 7
            axes[e.axis] = f710.get_axis(e.axis)
        elif e.type == pygame.locals.JOYHATMOTION:  # 9
            hats_x[e.hat], hats_y[e.hat] = f710.get_hats(e.hat)
        elif e.type == pygame.locals.JOYBUTTONDOWN:  # 10
            buttons[e.button] = 1
        elif e.type == pygame.locals.JOYBUTTONUP:  # 11
            buttons[e.button] = 0

        for index, item in enumerate(axes):
            print("axis(%s): %s" % (str(index), str(item)))
        for index, item in enumerate(buttons):
            print("button(%s): %s" % (str(index), str(item)))
        for index, item in enumerate(hats_x):
            print("hat(%s): %s, %s" %
                  (str(index), str(hats_x[index]), str(hats_y[index])))
