import numpy as np
import franky
from scipy.spatial.transform import Rotation as R
from spacemouse import Spacemouse
import time
from franky import Affine, RobotPose, CartesianWaypoint, CartesianWaypointMotion, JointWaypointMotion
import hydra.utils
from ik.ik_frankx import IKFrankY

# Constants
MOVE_INCREMENT = 0.0002
SPEED = 0.05  # [m/s]
FORCE = 20.0  # [N]
DT = 0.002  # [s]

# Initialize robot and gripper
robot = franky.Robot("172.16.0.2")
gripper = franky.Gripper("172.16.0.2")

robot.relative_dynamics_factor = 0.12
robot.velocity_rel = 0.2
robot.acceleration_rel = 0.1
robot.jerk_rel = 0.01
robot.recover_from_errors()

# Get initial pose
initial_pose = robot.current_cartesian_state.pose.end_effector_pose
initial_translation = np.array(initial_pose.translation)
initial_rotation = np.array(initial_pose.quaternion)

# Current pose starts as initial pose
current_translation = initial_translation.copy()
current_rotation = initial_rotation.copy()

waypoints = []
ik = IKFrankY


def move_robot(dx=0.0, dy=0.0, dz=0.0, drot=None):
    global current_translation, current_rotation

    # Update current translation and rotation
    current_translation += np.array([dx, dy, dz])

    if drot is not None:
        delta_rotation = R.from_euler('xyz', drot)
        current_rotation = (
            delta_rotation * R.from_quat(current_rotation)).as_quat()

    # Perform IK to get joint positions
    target_pos = current_translation
    target_orn = current_rotation
    current_q = np.array(robot.state.q)  # Current joint positions
    new_q = ik.inverse_kinematics(ik, target_pos, target_orn, current_q)

    # Create and append the waypoint with the updated pose
    waypoint = JointWaypointMotion([new_q], return_when_finished=False)
    waypoints.append(waypoint)


def execute_waypoints():
    global waypoints
    if waypoints:
        # Execute JointWaypointMotion
        motion = JointWaypointMotion(waypoints, return_when_finished=False)
        print("Executing waypoints", motion)
        # success = robot.move(motion, asynchronous=True)
        # if not success:
        #     robot.recover_from_errors()
        # waypoints = []  # Clear waypoints after execution


def toggle_gripper_state():
    current_width = gripper.width
    if current_width > 0.01:  # Assuming gripper is open if width > 1cm
        move_future = gripper.move_async(0.0, SPEED)  # Close the gripper
    else:
        move_future = gripper.open_async(SPEED)  # Open the gripper

    # Wait for the movement to complete with a timeout
    if move_future.wait(2):  # Wait for up to 1 second
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
        print(f"Translation: {dpos}, Rotation: {drot_xyz}")

        if sm.is_button_pressed(0):
            toggle_gripper_state()

        elif sm.is_button_pressed(1):
            grasp_object()

        elif sm.is_button_pressed(2):
            release_object()

        move_robot(dpos[0], dpos[1], dpos[2], drot_xyz)

        # Execute waypoints at a defined interval
        if len(waypoints) >= 10:  # Adjust the threshold as needed
            execute_waypoints()

        time.sleep(DT)

    # Execute any remaining waypoints when stopping
    execute_waypoints()
