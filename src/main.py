import time
import pybullet as p

from simulation.world import World
from control.ik_controller import IKController
from control.gripper_controller import GripperController
from planning.pick_place_task import PickPlaceTask


def get_panda_ee_link_index(robot_id: int) -> int:
    # find the index of the end-effector link based on name
    candidates = {"panda_hand", "panda_link8"}
    for i in range(p.getNumJoints(robot_id)):
        link_name = p.getJointInfo(robot_id, i)[12].decode("utf-8")
        if link_name in candidates:
            return i
    return p.getNumJoints(robot_id) - 1


def main():
    # initialize the simulation environment
    world = World()
    try:
        world.setup()

        # robot configuration and link identification
        robot_id = world.panda_id
        arm_joints = [0, 1, 2, 3, 4, 5, 6]
        ee_link = get_panda_ee_link_index(robot_id)

        # initialize inverse kinematics controller
        ik = IKController(
            world=world,
            robot_id=robot_id,
            ee_link=ee_link,
            arm_joints=arm_joints,
            max_force=200.0,
            max_velocity=1.0,
        )

        gripper = GripperController(robot_id) # initialize gripper controller

        task = PickPlaceTask(world, ik, gripper, use_attach=True) # initialize the high level task manager

        task.run(place_xy=(0.45, 0.20)) # execute the pick and place sequence

        # keep simulation open to view the result
        for _ in range(1200):
            world.step()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        pass
    finally:
        world.disconnect() # clean up and close connection


if __name__ == "__main__":
    main()