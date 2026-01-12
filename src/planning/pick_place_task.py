import pybullet as p
from control.ik_controller import IKController
from control.gripper_controller import GripperController
from utils.safety import safe_heights, workspace_limits, clamp_xyz


class PickPlaceTask:
    # manage high level logic for the pick and place operation
    def __init__(
        self,
        world,
        ik: IKController,
        gripper: GripperController,
        use_attach: bool = True,
    ):
        self.world = world
        self.ik = ik
        self.gripper = gripper
        self.use_attach = use_attach # option to use a constraint for stable grasping
        self._constraint_id = None

        # initialize workspace limits based on table height
        self.table_z = world.get_table_z()
        self.safe_z, self.pick_z = safe_heights(self.table_z)
        self.xlim, self.ylim, self.zlim = workspace_limits(self.table_z)

    def _clamp(self, target):
        # restrict target coordinates to safe workspace limits
        return clamp_xyz(target, self.xlim, self.ylim, self.zlim)

    def _dist(self, a, b) -> float:
        # calculate euclidean distance between two points
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _is_grasp_close_enough(self, thresh: float = 0.06) -> bool:
        # check if the gripper is close enough to the object to grasp
        pose = self.world.get_cube_pose()
        if pose is None:
            return False
        cube_pos, _ = pose
        ee_pos = p.getLinkState(self.ik.robot_id, self.ik.ee_link)[4]
        return self._dist(list(ee_pos), list(cube_pos)) < thresh

    def _attach_cube(self):
        # create a constraint to simulate a perfect grasp
        if not self._is_grasp_close_enough():
            return

        if not self.use_attach:
            return
        pose = self.world.get_cube_pose()
        if pose is None:
            return
        pos, orn = pose
        ee_state = p.getLinkState(self.ik.robot_id, self.ik.ee_link)
        ee_pos = ee_state[4]
        ee_orn = ee_state[5]

        # create a fixed constraint between the gripper and the cube
        self._constraint_id = p.createConstraint(
            parentBodyUniqueId=self.ik.robot_id,
            parentLinkIndex=self.ik.ee_link,
            childBodyUniqueId=self.world.cube_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0.02],
            childFramePosition=[0, 0, 0],
        )

        p.changeConstraint(self._constraint_id, maxForce=200)

    # release the constraint, dropping the object
    def _detach_cube(self):
        if self._constraint_id is not None:
            p.removeConstraint(self._constraint_id)
            self._constraint_id = None

    # execute full pick and place sequence
    def run(self, place_xy=(0.45, 0.20)):
        cube_pose = self.world.get_cube_pose()
        if cube_pose is None:
            return

        cube_pos, _ = cube_pose
        cx, cy, cz = cube_pos

        # calculate waypoints
        cube_top_z = cz + 0.02
        pre_grasp_z = self.safe_z
        grasp_z = max(self.table_z + 0.02, min(cube_top_z, self.table_z + 0.08))

        # clamp all points to ensure safety
        pre_grasp = self._clamp([cx, cy, pre_grasp_z])
        grasp = self._clamp([cx, cy, grasp_z])
        lift = self._clamp([cx, cy, self.safe_z])

        px, py = place_xy
        pre_place = self._clamp([px, py, self.safe_z])
        place = self._clamp([px, py, self.pick_z])
        retreat = self._clamp([px, py, self.safe_z])

        # execute sequence
        self.gripper.open()

        # approach
        self.ik.move_to(pre_grasp, duration_s=1.5, tol=0.02, settle_steps=30)
        self.ik.move_to(grasp, duration_s=1.8, tol=0.015, settle_steps=60)

        # grasp
        self.gripper.close(width=0.01, force=120.0, steps=240)
        self._attach_cube()

        # move to place
        self.ik.move_to(lift, duration_s=1.5, tol=0.02, settle_steps=40)
        self.ik.move_to(pre_place, duration_s=2.0, tol=0.02, settle_steps=40)
        self.ik.move_to(place, duration_s=1.5, tol=0.02, settle_steps=30)

        # release
        self._detach_cube()
        self.gripper.open()

        # retreat
        self.ik.move_to(retreat, duration_s=1.5, tol=0.02, settle_steps=40)