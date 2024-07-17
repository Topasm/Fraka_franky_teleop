import numpy as np
import time
import panda_py.controllers
from scipy.spatial.transform import Rotation as R
import panda_py
from spacemouse import Spacemouse

# Constants
MOVE_INCREMENT = 0.01
SPEED = 0.05  # [m/s]
FORCE = 20.0  # [N]
DT = 0.01  # [s]

# Initialize robot and gripper
hostname = '172.16.0.2'
robot = panda_py.Panda(hostname)
gripper = panda_py.libfranka.Gripper(hostname)


robot.recover()

# Get initial pose
current_translation = robot.get_position()
current_rotation = robot.get_orientation()

robot.move_to_start()


def main():
    print("Use Spacemouse to control the robot in Cartesian space.")
    print("Press button 0 to toggle gripper state (open/close).")
    print("Press button 1 to grasp an object and button 2 to release.")

    with Spacemouse(deadzone=0.3) as sm:
        running = True
        while running:
            start_time = time.time()

            sm_state = sm.get_motion_state_transformed()
            dpos = sm_state[:3] * MOVE_INCREMENT
            drot_xyz = sm_state[3:] * MOVE_INCREMENT
            drot_xyz = np.array([drot_xyz[0], drot_xyz[1], -drot_xyz[2]])

            # Update current pose
            global current_translation, current_rotation
            current_translation += np.array([dpos[0], dpos[1], dpos[2]])

            if drot_xyz is not None:
                delta_rotation = R.from_euler('xyz', drot_xyz)
                current_rotation = (
                    delta_rotation * R.from_quat(current_rotation)).as_quat()

            # Convert the current pose to the required format
            positions = [current_translation.reshape(3, 1)]
            orientations = [current_rotation.reshape(4, 1)]

            # Move the robot to the new pose
            robot.move_to_pose(
                positions=positions,
                orientations=orientations,
                speed_factor=0.2,
                damping_ratio=1,
                nullspace_stiffness=15,
                dq_threshold=0.001,
                success_threshold=0.01
            )

            # Handle gripper state changes
            if sm.is_button_pressed(0):
                current_width = gripper.get_gripper_width()
                if current_width > 0.01:  # Assuming gripper is open if width > 1cm
                    gripper.move(0.0, speed=SPEED)  # Close the gripper
                else:
                    gripper.open(speed=SPEED)  # Open the gripper

            elif sm.is_button_pressed(1):
                success = gripper.grasp(0.0, speed=SPEED, force=FORCE,
                                        epsilon_inner=0.005, epsilon_outer=1.0)
                if success:
                    print("Grasp successful")
                else:
                    print("Grasp failed")

            elif sm.is_button_pressed(2):
                success = gripper.open(speed=SPEED)
                if success:
                    print("Release successful")
                else:
                    print("Release failed")

            end_time = time.time()
            loop_duration = end_time - start_time
            loop_frequency = 1.0 / loop_duration
            print(f"Loop frequency: {loop_frequency:.2f} Hz")

            time.sleep(DT)


if __name__ == "__main__":
    main()
