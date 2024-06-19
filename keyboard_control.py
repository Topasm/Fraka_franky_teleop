import time
import franky
import keyboard

robot = franky.Robot("172.16.0.2")
gripper = franky.Gripper("172.16.0.2")

robot.relative_dynamics_factor = 0.05
speed = 0.02  # [m/s]
force = 20.0  # [N]


def move_robot(direction):
    if direction == 'w':
        motion = franky.CartesianMotion(franky.Affine(
            [0.01, 0.0, 0.0]), franky.ReferenceType.Relative)
    elif direction == 's':
        motion = franky.CartesianMotion(franky.Affine(
            [-0.01, 0.0, 0.0]), franky.ReferenceType.Relative)
    elif direction == 'd':
        motion = franky.CartesianMotion(franky.Affine(
            [0.0, 0.01, 0.0]), franky.ReferenceType.Relative)
    elif direction == 'a':
        motion = franky.CartesianMotion(franky.Affine(
            [0.0, -0.01, 0.0]), franky.ReferenceType.Relative)
    elif direction == 'x':
        motion = franky.CartesianMotion(franky.Affine(
            [0.0, 0.0, 0.01]), franky.ReferenceType.Relative)
    elif direction == 'z':
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


print("Use WASD keys to control the robot in Cartesian space.")
print("Use X to move up and Z to move down in the Z-axis.")
print("Press SPACE to toggle gripper state (open/close).")
print("Press G to grasp an object and R to release. Press ESC to exit.")
while True:
    if keyboard.is_pressed('w'):
        move_robot('w')
        time.sleep(0.1)  # Delay to avoid multiple moves per key press
    elif keyboard.is_pressed('s'):
        move_robot('s')
        time.sleep(0.1)
    elif keyboard.is_pressed('a'):
        move_robot('a')
        time.sleep(0.1)
    elif keyboard.is_pressed('d'):
        move_robot('d')
        time.sleep(0.1)
    elif keyboard.is_pressed('x'):
        move_robot('x')
        time.sleep(0.1)
    elif keyboard.is_pressed('z'):
        move_robot('z')
        time.sleep(0.1)
    elif keyboard.is_pressed('g'):
        grasp_object()
        time.sleep(0.5)  # Delay to avoid multiple actions on single press
    elif keyboard.is_pressed('r'):
        release_object()
        time.sleep(0.5)
    elif keyboard.is_pressed('esc'):
        break
