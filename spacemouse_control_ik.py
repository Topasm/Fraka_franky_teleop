import ctypes
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import franky
from spacemouse import Spacemouse
import threading

# Constants
MOVE_INCREMENT = 0.001
SPEED = 0.05  # [m/s]
FORCE = 20.0  # [N]
DT = 0.001  # [s]

# Load the shared library
pandaik_lib = ctypes.CDLL('/usr/local/diana/lib/libPanda-IK.so')

# Define the argument types and return type
pandaik_lib.compute_inverse_kinematics_void.argtypes = [ctypes.POINTER(
    ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
pandaik_lib.compute_inverse_kinematics_void.restype = None

# Initialize robot and gripper
robot = franky.Robot("172.16.0.2")
gripper = franky.Gripper("172.16.0.2")

robot.relative_dynamics_factor = 0.5


robot.recover_from_errors()

# Get initial pose
initial_pose = robot.current_cartesian_state.pose.end_effector_pose
initial_translation = np.array(initial_pose.translation)
initial_rotation = np.array(initial_pose.quaternion)

# Current pose starts as initial pose
current_translation = initial_translation.copy()
current_rotation = initial_rotation.copy()

move_lock = threading.Lock()


def update_current_pose(dx=0.0, dy=0.0, dz=0.0, drot=None):
    global current_translation, current_rotation

    current_translation += np.array([dx, dy, dz])

    if drot is not None:
        delta_rotation = R.from_euler('xyz', drot)
        current_rotation = (
            delta_rotation * R.from_quat(current_rotation)).as_quat()


def compute_inverse_kinematics():
    target_pos = current_translation
    target_orn = current_rotation
    xyzrpy = np.concatenate((target_pos, R.from_quat(
        target_orn).as_euler('xyz', degrees=True)))

    current_q = np.array(robot.state.q)

    xyzrpy_c = (ctypes.c_double * 6)(*xyzrpy)
    q_actual_c = (ctypes.c_double * 7)(*current_q)
    output_c = (ctypes.c_double * 7)()

    pandaik_lib.compute_inverse_kinematics_void(xyzrpy_c, q_actual_c, output_c)

    return np.array(list(output_c))


def robot_move_thread(dx=0.0, dy=0.0, dz=0.0, drot=None):
    while True:
        with move_lock:
            new_q = compute_inverse_kinematics()
            motion = franky.JointMotion(new_q.tolist())

            success = robot.move(motion, asynchronous=False)

            if not success:
                robot.recover_from_errors()
        time.sleep(DT)


def move_robot(dx=0.0, dy=0.0, dz=0.0, drot=None):
    with move_lock:
        update_current_pose(dx, dy, dz, drot)


def toggle_gripper_state():
    current_width = gripper.width
    if current_width > 0.01:  # Assuming gripper is open if width > 1cm
        move_future = gripper.move_async(0.0, SPEED)  # Close the gripper
    else:
        move_future = gripper.open_async(SPEED)  # Open the gripper

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


def main():
    print("Use Spacemouse to control the robot in Cartesian space.")
    print("Press button 0 to toggle gripper state (open/close).")
    print("Press button 1 to grasp an object and button 2 to release.")

    move_thread = threading.Thread(target=robot_move_thread, daemon=True)
    move_thread.start()

    with Spacemouse(deadzone=0.3) as sm:
        running = True
        while running:
            start_time = time.time()

            sm_state = sm.get_motion_state_transformed()
            dpos = sm_state[:3] * MOVE_INCREMENT
            drot_xyz = sm_state[3:] * MOVE_INCREMENT
            drot_xyz = np.array([-drot_xyz[0], drot_xyz[1], drot_xyz[2]])
            # print(f"Translation: {dpos}, Rotation: {drot_xyz}")

            if sm.is_button_pressed(0):
                toggle_gripper_state()

            elif sm.is_button_pressed(1):
                grasp_object()

            elif sm.is_button_pressed(2):
                release_object()

            move_robot(dpos[0], dpos[1], dpos[2], drot_xyz)

            end_time = time.time()
            loop_duration = end_time - start_time
            loop_frequency = 1.0 / loop_duration
            # print(f"Loop frequency: {loop_frequency:.2f} Hz")

            time.sleep(DT)


if __name__ == "__main__":
    main()
