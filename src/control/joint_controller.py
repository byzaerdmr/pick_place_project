import pybullet as p


def set_arm_joints(
    robot_id: int,
    joint_indices: list[int],
    target_positions: list[float],
    max_force: float = 200.0,
    position_gain: float = 0.10,
    velocity_gain: float = 1.00,
    max_velocity: float = 1.0,
) -> None:
    kwargs = dict(
        controlMode=p.POSITION_CONTROL,
        targetPositions=target_positions,
        forces=[max_force] * len(joint_indices),
        positionGains=[position_gain] * len(joint_indices),
        velocityGains=[velocity_gain] * len(joint_indices),
    )

    try:
        p.setJointMotorControlArray(
            robot_id,
            joint_indices,
            **kwargs,
            maxVelocities=[max_velocity] * len(joint_indices),
        )
    except TypeError:
        p.setJointMotorControlArray(
            robot_id,
            joint_indices,
            **kwargs,
        )
