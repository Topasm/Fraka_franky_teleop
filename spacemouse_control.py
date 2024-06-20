import numpy as np
from franky import Affine, CartesianMotion, Robot, ReferenceType
from scipy.spatial.transform import Rotation as R
from spacemouse import Spacemouse
import time


def activate(robot, duration=10):
    current_pose = robot.current_cartesian_state.pose
    motion = CartesianMotion(Affine(
        [current_pose.translation, current_pose.rotation]), ReferenceType.Absolute)
    robot.move(motion, dynamic=True)


if __name__ == "__main__":
    robot = Robot("172.16.0.3")
    robot.recover_from_errors()
    robot.move_to_joint_positions(
        [0, -0.785, 0, -2.356, 0, 1.571, 0], time_to_go=5)  # Reset joints

    print('Ready...')

    dt = 0.02

    activate(robot, duration=50)
    print('Activated...')

    max_pos_speed = 0.15
    max_rot_speed = 0.25

    current_pose = robot.current_cartesian_state.pose
    target_pose = Affine([current_pose.translation, current_pose.rotation])
    gripper_open = True

    with Spacemouse(deadzone=0.3) as sm:
        while True:
            sm_state = sm.get_motion_state_transformed()
            dpos = sm_state[:3] * (max_pos_speed * dt)
            drot_xyz = sm_state[3:] * (max_rot_speed * dt)

            if not sm.is_button_pressed(0):
                # translation mode
                drot_xyz[:] = 0
            else:
                dpos[:] = 0

            if sm.is_button_pressed(1):
                if gripper_open:
                    robot.close_gripper()
                    gripper_open = False
                else:
                    robot.open_gripper()
                    gripper_open = True

            drot = R.from_euler('xyz', drot_xyz)
            target_pose.translation += dpos
            target_pose.rotation = (
                drot * R.from_matrix(target_pose.rotation)).as_matrix()

            # Move the robot to the target pose
            motion = CartesianMotion(Affine(
                [target_pose.translation, target_pose.rotation]), ReferenceType.Absolute)
            robot.move(motion, asynchronous=True)

            # Ensure the loop runs at the desired rate
            time.sleep(dt)
