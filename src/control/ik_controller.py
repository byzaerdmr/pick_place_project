import pybullet as p
from .joint_controller import set_arm_joints

def move_ee_ik(
    robot_id: int,
    ee_link: int,
    arm_joints: list[int],
    target_pos: list[float],
    target_orn: list[float] | None = None,
    steps: int = 240,
    max_force: float = 200.0,
) -> None:
    if target_orn is None:
        q = p.calculateInverseKinematics(robot_id, ee_link, targetPosition=target_pos)
    else:
        q = p.calculateInverseKinematics(
            robot_id, ee_link, targetPosition=target_pos, targetOrientation=target_orn
        )

    target_positions = [q[j] for j in arm_joints]

    for _ in range(steps):
        set_arm_joints(robot_id, arm_joints, target_positions, max_force=max_force)
        p.stepSimulation()
