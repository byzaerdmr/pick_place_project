import time
import math
import pybullet as p
from simulation.world import World
from control.ik_controller import IKController
from control.gripper_controller import GripperController
from control.joint_controller import set_arm_joints

# Constants
EE_LINK = 11
ARM_JOINTS = [0, 1, 2, 3, 4, 5, 6]
MOVE_STEP = 0.0025
GRIPPER_INIT = 0.04
GRIPPER_MAX = 0.08
GRIPPER_MIN = 0.0
GRIPPER_FORCE = 150
GRIPPER_STEP = 0.0015

# Camera manager
class CameraManager:
    def __init__(self, world, robot_id, ee_link):
        self.world = world
        self.robot_id = robot_id
        self.ee_link = ee_link
        self.view_mode = 0  # 0: main, 1: side, 2: fps view

    def handle_input(self, keys):
        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
            self.view_mode = (self.view_mode + 1) % 3
            self._set_static_view()
        if self.view_mode == 2:
            self._update_fps_view()

    def _set_static_view(self):
        if self.view_mode == 0:
            self.world.camera_yaw = 10
            p.resetDebugVisualizerCamera(
                1.0, self.world.camera_yaw, -40, [0.65, 0, 0.75])
        elif self.view_mode == 1:
            self.world.camera_yaw = 90
            p.resetDebugVisualizerCamera(
                1.2, self.world.camera_yaw, -30, [0.4, 0, 0.5])

    # fps camera mode
    def _update_fps_view(self):
        ee_state = p.getLinkState(self.robot_id, self.ee_link)
        ee_pos = ee_state[4]
        camera_target = [ee_pos[0] + 0.15, ee_pos[1], ee_pos[2] - 0.1]

        self.world.camera_yaw = 0
        p.resetDebugVisualizerCamera(
            cameraDistance=0.40,
            cameraYaw=self.world.camera_yaw,
            cameraPitch=-70,
            cameraTargetPosition=camera_target)

    def yaw_rad(self):
        return math.radians(self.world.camera_yaw)

# Helper functions
def camera_relative_motion(keys, yaw_rad, step):
    dx, dy = 0.0, 0.0 # compute motion in camera frame

    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        dy += step
    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        dy -= step
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        dx -= step
    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        dx += step

    wx = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
    wy = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    return wx, wy

def solve_ik(robot_id, ee_link, target_pos, target_orn):
    return p.calculateInverseKinematics(
        robot_id,
        ee_link,
        target_pos,
        target_orn)

# Main
def main():
    world = World()
    try:
        world.setup()
        robot_id = world.panda_id

        ik = IKController(world, robot_id, EE_LINK, ARM_JOINTS)
        gripper = GripperController(robot_id)
        camera = CameraManager(world, robot_id, EE_LINK)

        target_pos = ik.ee_pos()
        gripper_width = GRIPPER_INIT

        while p.isConnected():
            keys = p.getKeyboardEvents()

            # camera
            camera.handle_input(keys)
            yaw_rad = camera.yaw_rad()

            # xy motion
            dx, dy = camera_relative_motion(keys, yaw_rad, MOVE_STEP)
            target_pos[0] += dx
            target_pos[1] += dy

            # z motion
            if p.B3G_PAGE_UP in keys and keys[p.B3G_PAGE_UP] & p.KEY_IS_DOWN:
                target_pos[2] += MOVE_STEP
            if p.B3G_PAGE_DOWN in keys and keys[p.B3G_PAGE_DOWN] & p.KEY_IS_DOWN:
                target_pos[2] -= MOVE_STEP

            # gripper
            if p.B3G_HOME in keys and keys[p.B3G_HOME] & p.KEY_IS_DOWN:
                gripper_width = min(GRIPPER_MAX, gripper_width + GRIPPER_STEP)
            if p.B3G_END in keys and keys[p.B3G_END] & p.KEY_IS_DOWN:
                gripper_width = max(GRIPPER_MIN, gripper_width - GRIPPER_STEP)

            # ik+control, apply joint and gripper positions
            joint_poses = solve_ik(
                robot_id,
                EE_LINK,
                target_pos,
                ik.down_orn)

            set_arm_joints(
                robot_id,
                ARM_JOINTS,
                [joint_poses[i] for i in range(7)])

            p.setJointMotorControlArray(
                robot_id,
                gripper.finger_joints,
                p.POSITION_CONTROL,
                targetPositions=[gripper_width / 2, gripper_width / 2],
                forces=[GRIPPER_FORCE, GRIPPER_FORCE])

            world.step()
            time.sleep(1.0 / 240.0)

    finally:
        world.disconnect()

if __name__ == "__main__":
    main()