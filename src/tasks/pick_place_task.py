import pybullet as p
import time
from utils.safety import safe_heights, workspace_limits, clamp_xyz

class PickPlaceTask:
    def __init__(self, world, ik, gripper, use_attach: bool = True):
        self.world = world
        self.ik = ik
        self.gripper = gripper
        self.use_attach = use_attach
        self._constraint_id = None

        # direct access to world variables
        self.table_z = world.table_z
        self.safe_z, self.pick_z = safe_heights(self.table_z)
        self.xlim, self.ylim, self.zlim = workspace_limits(self.table_z)

    def _clamp(self, target):
        return clamp_xyz(target, self.xlim, self.ylim, self.zlim)

    def _attach_cube(self):
        if not self.use_attach: return
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

    def _detach_cube(self):
        if self._constraint_id is not None:
            p.removeConstraint(self._constraint_id)
            self._constraint_id = None

    def run(self, place_xy=(0.45, 0.20)):
        # get current cube position
        cube_pos, _ = p.getBasePositionAndOrientation(self.world.cube_id)
        cx, cy, cz = cube_pos
        
        # motion waypoints
        pre_grasp = self._clamp([cx, cy, self.safe_z])
        grasp = self._clamp([cx, cy, self.table_z + 0.035])
        lift = self._clamp([cx, cy, self.safe_z + 0.1])
        px, py = place_xy
        pre_place = self._clamp([px, py, self.safe_z + 0.1])
        place = self._clamp([px, py, self.table_z + 0.05])

        # start sequence
        self.gripper.open()
        self.ik.move_to(pre_grasp, duration_s=1.2)
        self.ik.move_to(grasp, duration_s=1.0)
        self.gripper.close(width=0.015)
        time.sleep(0.6)
        self._attach_cube()
        
        self.ik.move_to(lift, duration_s=1.0)
        self.ik.move_to(pre_place, duration_s=1.5)
        self.ik.move_to(place, duration_s=1.0)
        
        self._detach_cube()
        self.gripper.open()
        self.ik.move_to(pre_place, duration_s=1.0)