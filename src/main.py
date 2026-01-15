import time
import math
import pybullet as p
from simulation.world import World
from control.ik_controller import IKController
from control.gripper_controller import GripperController
from control.joint_controller import set_arm_joints

def main():
    world = World()
    try:
        world.setup()
        robot_id = world.panda_id
        ee_link = 11 
        ik = IKController(world, robot_id, ee_link, [0,1,2,3,4,5,6])
        gripper = GripperController(robot_id)
        
        target_pos = ik.ee_pos()
        gripper_width = 0.04
        d_val = 0.0025 # speed of movement
        
        while p.isConnected():
            keys = p.getKeyboardEvents()

            # compute motion in camera frame
            yaw_rad = math.radians(world.camera_yaw)
            dx, dy = 0, 0
            
            # directions
            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                dy = d_val
            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                dy = -d_val
                
            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                dx = -d_val 
            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                dx = d_val 

            # rotation
            target_pos[0] += dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
            target_pos[1] += dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

            # move up/down
            if p.B3G_PAGE_UP in keys and keys[p.B3G_PAGE_UP] & p.KEY_IS_DOWN: target_pos[2] += d_val
            if p.B3G_PAGE_DOWN in keys and keys[p.B3G_PAGE_DOWN] & p.KEY_IS_DOWN: target_pos[2] -= d_val
            
            # control gripper
            if p.B3G_HOME in keys and keys[p.B3G_HOME] & p.KEY_IS_DOWN:
                gripper_width = min(0.08, gripper_width + 0.0015)
            if p.B3G_END in keys and keys[p.B3G_END] & p.KEY_IS_DOWN:
                gripper_width = max(0.00, gripper_width - 0.0015)

            # apply joint and gripper positions
            joint_poses = p.calculateInverseKinematics(robot_id, ee_link, target_pos, ik.down_orn)
            set_arm_joints(robot_id, [0,1,2,3,4,5,6], [joint_poses[i] for i in range(7)])
            p.setJointMotorControlArray(robot_id, gripper.finger_joints, p.POSITION_CONTROL,
                                        targetPositions=[gripper_width/2, gripper_width/2], forces=[150,150])

            world.step()
            time.sleep(1.0 / 240.0)

    finally: 
        world.disconnect()

if __name__ == "__main__":
    main()