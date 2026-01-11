import pybullet as p

def set_arm_joints(
    robot_id: int,
    joint_indices: list[int],
    target_positions: list[float],
    max_force: float = 200.0,
    position_gain: float = 0.15,
    velocity_gain: float = 1.0,
) -> None:
    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=target_positions,
        forces=[max_force] * len(joint_indices),
        positionGains=[position_gain] * len(joint_indices),
        velocityGains=[velocity_gain] * len(joint_indices),
    )
