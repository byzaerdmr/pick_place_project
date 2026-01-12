import pybullet as p
import time


class GripperController:
    # handle low level control of panda robot's gripper
    def __init__(self, robot_id: int):
        self.robot_id = robot_id
        # automatically identify finger joint indices
        self.finger_joints = self._find_finger_joints()

    def _find_finger_joints(self) -> list[int]:
        # locate indices of the two finger joints by name
        names = {"panda_finger_joint1", "panda_finger_joint2"}
        joints = []
        for i in range(p.getNumJoints(self.robot_id)):
            jname = p.getJointInfo(self.robot_id, i)[1].decode("utf-8")
            if jname in names:
                joints.append(i)

        # fallback to default indices if names are not found
        if len(joints) != 2:
            joints = [9, 10]
        return joints

    def open(self, width: float = 0.08, force: float = 60.0, steps: int = 120):
        # open the gripper fingers to the specified width
        target = width / 2.0 # calculate symmetric target for each finger
        for _ in range(steps):
            p.setJointMotorControlArray(
                self.robot_id,
                self.finger_joints,
                p.POSITION_CONTROL,
                targetPositions=[target, target],
                forces=[force, force],
            )
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    def close(self, width: float = 0.01, force: float = 120.0, steps: int = 240):
        # close the gripper fingers to grasp an object
        target = width / 2.0
        for _ in range(steps):
            p.setJointMotorControlArray(
                self.robot_id,
                self.finger_joints,
                p.POSITION_CONTROL,
                targetPositions=[target, target],
                forces=[force, force],
            )

            p.stepSimulation()
            time.sleep(1.0 / 240.0)