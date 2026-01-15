import pybullet as p
import math
from .joint_controller import set_arm_joints

# math functions
def _vec_sub(a, b): return [a[i] - b[i] for i in range(3)]
def _vec_add(a, b): return [a[i] + b[i] for i in range(3)]
def _vec_mul(a, s): return [a[i] * s for i in range(3)]
def _dist(a, b): return sum((a[i] - b[i])**2 for i in range(3))**0.5

class IKController:
    def __init__(
        self,
        world,
        robot_id: int,
        ee_link: int,
        arm_joints: list[int],
        max_force: float = 200.0,
        max_velocity: float = 1.0,
    ):
        self.world = world
        self.robot_id = robot_id
        self.ee_link = ee_link
        self.arm_joints = arm_joints
        self.max_force = max_force
        self.max_velocity = max_velocity
        
        self.down_orn = p.getQuaternionFromEuler([math.pi, 0, -math.pi / 4])

    def ee_pos(self) -> list[float]:
        # position of robot's endpoint
        return list(p.getLinkState(self.robot_id, self.ee_link)[4])

    def hold_current_pose(self):
        # leave robot where it is
        current_joints = [p.getJointState(self.robot_id, i)[0] for i in self.arm_joints]
        set_arm_joints(self.robot_id, self.arm_joints, current_joints, max_force=self.max_force)

    def move_to(
        self,
        target_pos: list[float],
        target_orn: list[float] | None = None,
        duration_s: float = 1.5,
        tol: float = 0.005,
        settle_steps: int = 40,
    ) -> float:

        start = self.ee_pos()
        steps = max(1, int(duration_s * 240))
        
        if target_orn is None:
            target_orn = self.down_orn

        last_err = 0.0
        for t in range(steps):
            alpha = (t + 1) / steps
            interp_pos = _vec_add(start, _vec_mul(_vec_sub(target_pos, start), alpha))

            # ik solver
            joint_poses = p.calculateInverseKinematics(
                self.robot_id,
                self.ee_link,
                targetPosition=interp_pos,
                targetOrientation=target_orn,
                maxNumIterations=100,
                residualThreshold=1e-5
            )

            target_joints = [joint_poses[i] for i in range(len(self.arm_joints))]
            set_arm_joints(
                self.robot_id,
                self.arm_joints,
                target_joints,
                max_force=self.max_force,
                max_velocity=self.max_velocity,
            )

            self.world.step()
            last_err = _dist(self.ee_pos(), interp_pos)

        # waiting for shake
        for _ in range(settle_steps):
            self.world.step()

        return last_err