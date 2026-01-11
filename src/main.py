import time
import pybullet as p
from simulation.world import World
from control.ik_controller import IKController


def get_panda_ee_link_index(robot_id: int) -> int:
    candidates = {"panda_hand", "panda_link8"}
    for i in range(p.getNumJoints(robot_id)):
        link_name = p.getJointInfo(robot_id, i)[12].decode("utf-8")
        if link_name in candidates:
            return i
    return p.getNumJoints(robot_id) - 1


def main():
    world = World()
    try:
        world.setup()

        robot_id = world.panda_id
        arm_joints = [0, 1, 2, 3, 4, 5, 6]
        ee_link = get_panda_ee_link_index(robot_id)

        ik = IKController(
            world=world,
            robot_id=robot_id,
            ee_link=ee_link,
            arm_joints=arm_joints,
            max_force=200.0,
            max_velocity=1.0,
        )

        targets = [
            [0.55, 0.00, 0.85],
            [0.50, 0.15, 0.85],
            [0.50, -0.15, 0.85],
        ]

        for tgt in targets:
            err = ik.move_to(tgt, duration_s=1.8, tol=0.02, settle_steps=40)
            print(f"target={tgt} err={err:.4f}")
            for _ in range(60):
                world.step()
                time.sleep(1.0 / 240.0)

        for _ in range(2400):
            world.step()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        pass
    finally:
        world.disconnect()


if __name__ == "__main__":
    main()
