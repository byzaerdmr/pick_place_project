import time
import math
import pybullet as p
import pybullet_data


def get_panda_ee_link_index(robot_id: int) -> int:
    ee_candidates = {"panda_hand", "panda_link8"}
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[12].decode("utf-8")
        if name in ee_candidates:
            return i
    return p.getNumJoints(robot_id) - 1


def get_panda_arm_joint_indices(robot_id: int):
    joints = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        joint_type = info[2]
        joint_name = info[1].decode("utf-8")
        if joint_type == p.JOINT_REVOLUTE and joint_name.startswith("panda_joint"):
            joints.append(i)
    return joints[:7]


def set_arm_joints(robot_id: int, joint_indices, target_positions):
    for j, q in zip(joint_indices, target_positions):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=float(q),
            force=250,
            positionGain=0.08,
            velocityGain=1.0,
        )


def move_ee_ik(robot_id: int, ee_link: int, arm_joints, target_pos, target_orn=None, steps=480):
    if target_orn is None:
        target_orn = p.getQuaternionFromEuler([math.pi, 0.0, 0.0])

    ik = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=ee_link,
        targetPosition=target_pos,
        targetOrientation=target_orn,
        maxNumIterations=200,
        residualThreshold=1e-4,
    )

    q_arm = [ik[i] for i in range(len(arm_joints))]
    set_arm_joints(robot_id, arm_joints, q_arm)

    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1 / 240)


def main():
    p.connect(p.GUI)
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    p.loadURDF("cube_small.urdf", [0.5, 0.0, 0.02])

    ee_link = get_panda_ee_link_index(robot_id)
    arm_joints = get_panda_arm_joint_indices(robot_id)

    move_ee_ik(robot_id, ee_link, arm_joints, target_pos=[0.4, 0.0, 0.5], steps=360)
    move_ee_ik(robot_id, ee_link, arm_joints, target_pos=[0.55, 0.05, 0.35], steps=360)
    move_ee_ik(robot_id, ee_link, arm_joints, target_pos=[0.55, 0.05, 0.25], steps=360)
    move_ee_ik(robot_id, ee_link, arm_joints, target_pos=[0.55, 0.05, 0.45], steps=360)

    while True:
        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()
