import numpy as np
import franky
from scipy.spatial.transform import Rotation as R
from spacemouse import Spacemouse
import time

# Constants
MOVE_INCREMENT = 0.1
SPEED = 0.05  # [m/s]
FORCE = 20.0  # [N]
DT = 0.01  # Loop interval in seconds

# Initialize robot and gripper
robot = franky.Robot("172.16.0.2")
gripper = franky.Gripper("172.16.0.2")

robot.relative_dynamics_factor = 0.05


def move_robot(dx=0.0, dy=0.0, dz=0.0, drot=None):
    current_pose = robot.current_cartesian_state.pose
    end_effector_pose = current_pose.end_effector_pose  # Access the end_effector_pose

    # Extract translation and rotation from end_effector_pose
    translation = np.array(end_effector_pose.translation)
    rotation = np.array(end_effector_pose.quaternion)

    new_translation = translation + np.array([dx, dy, dz])

    if drot is not None:
        current_rotation = R.from_quat(rotation)
        delta_rotation = R.from_euler('xyz', drot)
        new_rotation = (delta_rotation * current_rotation).as_quat()
    else:
        new_rotation = rotation

    # Correctly format the translation and quaternion for Affine constructor
    motion = franky.CartesianMotion(
        franky.Affine(new_translation, new_rotation), franky.ReferenceType.Absolute)
    robot.move(motion, asynchronous=True)


def toggle_gripper_state():
    current_width = gripper.width
    if current_width > 0.01:  # Assuming gripper is open if width > 1cm
        move_future = gripper.move_async(0.0, SPEED)  # Close the gripper
    else:
        move_future = gripper.open_async(SPEED)  # Open the gripper

    # Wait for the movement to complete with a timeout
    if move_future.wait(1):  # Wait for up to 1 second
        if move_future.get():
            print("Toggle gripper state successful")
        else:
            print("Toggle gripper state failed")
    else:
        gripper.stop()
        print("Toggle gripper state timed out")


def grasp_object():
    grasp_future = gripper.grasp_async(
        0.0, SPEED, FORCE, epsilon_inner=0.005, epsilon_outer=1.0)

    if grasp_future.wait(1):  # Wait for up to 1 second
        if grasp_future.get():
            print("Grasp successful")
            return True
        else:
            print("Grasp failed")
            return False
    else:
        gripper.stop()
        print("Grasp timed out")
        return False


def release_object():
    release_future = gripper.open_async(SPEED)

    if release_future.wait(1):  # Wait for up to 1 second
        if release_future.get():
            print("Release successful")
        else:
            print("Release failed")
    else:
        gripper.stop()
        print("Release timed out")


print("Use Spacemouse to control the robot in Cartesian space.")
print("Press button 0 to toggle gripper state (open/close).")
print("Press button 1 to grasp an object and button 2 to release.")

with Spacemouse(deadzone=0.3) as sm:
    running = True
    while running:
        sm_state = sm.get_motion_state_transformed()
        dpos = sm_state[:3] * MOVE_INCREMENT
        drot_xyz = sm_state[3:] * MOVE_INCREMENT

        if sm.is_button_pressed(0):
            toggle_gripper_state()

        elif sm.is_button_pressed(1):
            grasp_object()

        elif sm.is_button_pressed(2):
            release_object()

        move_robot(dpos[0], dpos[1], dpos[2], drot_xyz)

        time.sleep(DT)
