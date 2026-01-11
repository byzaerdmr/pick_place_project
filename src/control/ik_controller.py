import pybullet as p
from .joint_controller import set_arm_joints


def _vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _vec_mul(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def _dist(a, b):
    d = _vec_sub(a, b)
    return (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) ** 0.5


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

    def ee_pos(self) -> list[float]:
        return list(p.getLinkState(self.robot_id, self.ee_link)[4])

    def move_to(
        self,
        target_pos: list[float],
        target_orn: list[float] | None = None,
        duration_s: float = 1.5,
        tol: float = 0.02,
        settle_steps: int = 30,
    ) -> float:
        start = self.ee_pos()
        steps = max(1, int(duration_s * 240))

        last_err = _dist(start, target_pos)

        for t in range(steps):
            alpha = (t + 1) / steps
            interp_pos = _vec_add(start, _vec_mul(_vec_sub(target_pos, start), alpha))

            if target_orn is None:
                q = p.calculateInverseKinematics(
                    self.robot_id, self.ee_link, targetPosition=interp_pos
                )
            else:
                q = p.calculateInverseKinematics(
                    self.robot_id,
                    self.ee_link,
                    targetPosition=interp_pos,
                    targetOrientation=target_orn,
                )

            target_joints = [q[j] for j in self.arm_joints]
            set_arm_joints(
                self.robot_id,
                self.arm_joints,
                target_joints,
                max_force=self.max_force,
                max_velocity=self.max_velocity,
            )

            self.world.step()

            cur = self.ee_pos()
            last_err = _dist(cur, interp_pos)

            if last_err <= tol:
                break

        for _ in range(settle_steps):
            self.world.step()

        return last_err
