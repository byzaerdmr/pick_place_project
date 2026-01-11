import time
from simulation.world import World
import pybullet as p
from control.ik_controller import move_ee_ik

def get_panda_ee_link_index(robot_id: int) -> int:
    candidates = {"panda_hand", "panda_link8"}
    for i in range(p.getNumJoints(robot_id)):
        link_name = p.getJointInfo(robot_id, i)[12].decode("utf-8")
        if link_name in candidates:
            return i
    return p.getNumJoints(robot_id) - 1


def main():
    # Create World instance
    world = World()

    try:
        world.setup()

        robot_id = world.panda_id
        arm_joints = [0, 1, 2, 3, 4, 5, 6]
        ee_link = get_panda_ee_link_index(robot_id)

        target = [0.55, 0.0, 0.85]
        move_ee_ik(robot_id, ee_link, arm_joints, target, steps=240)

        # Simulation loop
        for _ in range(2400):
            world.step()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        pass
    finally:
        world.disconnect()

if __name__ == "__main__":
    main()