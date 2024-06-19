import time
import franky
import pygame

# Initialize the joystick
pygame.init()
pygame.joystick.init()

# Check if joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick connected.")
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

robot = franky.Robot("172.16.0.2")
gripper = franky.Gripper("172.16.0.2")

robot.relative_dynamics_factor = 0.05
speed = 0.02  # [m/s]
force = 20.0  # [N]


def move_robot(direction):
    if direction == 'up':
        motion = franky.CartesianMotion(franky.Affine(
            [0.01, 0.0, 0.0]), franky.ReferenceType.Relative)
    elif direction == 'down':
        motion = franky.CartesianMotion(franky.Affine(
            [-0.01, 0.0, 0.0]), franky.ReferenceType.Relative)
    elif direction == 'right':
        motion = franky.CartesianMotion(franky.Affine(
            [0.0, 0.01, 0.0]), franky.ReferenceType.Relative)
    elif direction == 'left':
        motion = franky.CartesianMotion(franky.Affine(
            [0.0, -0.01, 0.0]), franky.ReferenceType.Relative)
    elif direction == 'down_z':
        motion = franky.CartesianMotion(franky.Affine(
            [0.0, 0.0, 0.01]), franky.ReferenceType.Relative)
    elif direction == 'up_z':
        motion = franky.CartesianMotion(franky.Affine(
            [0.0, 0.0, -0.01]), franky.ReferenceType.Relative)
    else:
        return
    robot.move(motion, asynchronous=True)


def toggle_gripper_state():
    current_width = gripper.width
    if current_width > 0.01:  # Assuming gripper is open if width > 1cm
        gripper.move(0.0, speed)  # Close the gripper
    else:
        gripper.open(speed)  # Open the gripper


def grasp_object():
    current_width = gripper.width
    # Move fingers to the current width
    success = gripper.move(current_width, speed)
    success &= gripper.grasp(0.0, speed, force, epsilon_outer=1.0)
    return success


def release_object():
    gripper.open(speed)


print("Use joystick to control the robot in Cartesian space.")
print("Press button 0 (A) to toggle gripper state (open/close).")
print("Press button 1 (B) to grasp an object and button 2 (X) to release. Press button 9 (Start) to exit.")
print("Use LT (axis 3) and LB (button 5) to move up and down in Z direction.")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            # Axis 0: left-right, Axis 1: forward-backward
            if joystick.get_axis(1) < -0.5:
                move_robot('up')
            elif joystick.get_axis(1) > 0.5:
                move_robot('down')
            if joystick.get_axis(0) < -0.5:
                move_robot('left')
            elif joystick.get_axis(0) > 0.5:
                move_robot('right')
            if joystick.get_axis(2) < -0.5:
                move_robot('down_z')

        elif event.type == pygame.JOYBUTTONDOWN:
            if joystick.get_button(0):  # Button A toggles gripper state
                toggle_gripper_state()
            elif joystick.get_button(1):  # Button B grasps object
                grasp_object()
            elif joystick.get_button(2):  # Button X releases object
                release_object()
            elif joystick.get_button(4):  # Button LB moves up in Z direction
                move_robot('up_z')
            elif joystick.get_button(9):  # Button Start exits the loop
                running = False
                break

pygame.quit()
