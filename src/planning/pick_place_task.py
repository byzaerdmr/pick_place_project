import pybullet as p
import time
from control.ik_controller import IKController
from control.gripper_controller import GripperController
from utils.safety import safe_heights, workspace_limits, clamp_xyz

class PickPlaceTask:
    # manage high level logic for the pick and place operation
    def __init__(self, world, ik: IKController, gripper: GripperController, use_attach: bool = True):
        self.world = world
        self.ik = ik
        self.gripper = gripper
        self.use_attach = use_attach # option to secure the object during transport
        self._constraint_id = None

        # initialize limits from world data
        self.table_z = world.get_table_z()
        self.safe_z, self.pick_z = safe_heights(self.table_z)
        self.xlim, self.ylim, self.zlim = workspace_limits(self.table_z)

    def _clamp(self, target):
        # restrict target coordinates to safe workspace limits
        return clamp_xyz(target, self.xlim, self.ylim, self.zlim)

    def _dist(self, a, b) -> float:
        # calculate euclidean distance between two points
        return sum((a[i] - b[i])**2 for i in range(3))**0.5

    def _is_grasp_close_enough(self, thresh: float = 0.08) -> bool:
        # verify if the gripper is correctly positioned over the cube
        pose = self.world.get_cube_pose()
        if pose is None: return False
        ee_pos = p.getLinkState(self.ik.robot_id, self.ik.ee_link)[4]
        return self._dist(list(ee_pos), list(pose[0])) < thresh

    def _attach_cube(self):
        # create a virtual bond between gripper and cube to prevent sliding
        if not self._is_grasp_close_enough() or not self.use_attach: return
        
        # lock cube to end-effector link
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
        p.changeConstraint(self._constraint_id, maxForce=150)

    def _detach_cube(self):
        # release the virtual bond for drop-off
        if self._constraint_id is not None:
            p.removeConstraint(self._constraint_id)
            self._constraint_id = None

    def run(self, place_xy=(0.45, 0.20)):
        # execute full mission sequence: pick, transport, and place
        cube_pose = self.world.get_cube_pose()
        if cube_pose is None: return

        cx, cy, cz = cube_pose[0]
        
        # calculate precise heights for each phase
        pre_grasp = self._clamp([cx, cy, self.safe_z + 0.05])
        grasp = self._clamp([cx, cy, self.table_z + 0.035])
        lift = self._clamp([cx, cy, self.safe_z + 0.1])
        
        px, py = place_xy
        pre_place = self._clamp([px, py, self.safe_z + 0.1])
        place = self._clamp([px, py, self.table_z + 0.05])

        # approach phase
        self.gripper.open()
        self.ik.move_to(pre_grasp, duration_s=1.2)
        self.ik.move_to(grasp, duration_s=1.0)
        
        # grasping phase with stabilization delay
        self.gripper.close(width=0.015)
        time.sleep(0.6) # allow physics to settle before attachment
        self._attach_cube()
        
        # transport phase
        self.ik.move_to(lift, duration_s=1.0)
        self.ik.move_to(pre_place, duration_s=1.5)
        self.ik.move_to(place, duration_s=1.0)
        
        # release phase
        self._detach_cube()
        self.gripper.open()
        self.ik.move_to(pre_place, duration_s=1.0)