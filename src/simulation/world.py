import pybullet as p
import pybullet_data


class World:
    def __init__(self):
        self.PANDA_ARM_JOINTS = [0, 1, 2, 3, 4, 5, 6]
        self.PANDA_HOME = [0.0, -0.4, 0.0, -2.2, 0.0, 2.0, 0.8]

        self.panda_id = None
        self.cube_id = None

        self.table_id = None
        self.table_z = 0.62

        self.cube_spawn = [0.55, 0.0, 0.70]
        self.cube_min_z = 0.58

    def setup(self):
        cid = p.connect(p.GUI)
        if cid < 0:
            raise RuntimeError("PyBullet GUI connection failed")

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        p.setTimeStep(1.0 / 240.0)
        p.setPhysicsEngineParameter(
            fixedTimeStep=1.0 / 240.0,
            numSolverIterations=150,
            numSubSteps=4,
            enableConeFriction=1,
            contactBreakingThreshold=0.001,
        )

        p.setRealTimeSimulation(0)

        p.loadURDF("plane.urdf")

        self.table_id = p.loadURDF(
            "table/table.urdf",
            basePosition=[0.6, 0.0, 0.0],
            useFixedBase=True,
        )

        aabb = p.getAABB(self.table_id)
        self.table_z = aabb[1][2]
        self.cube_min_z = self.table_z - 0.05

        self.panda_id = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[0.0, 0.0, 0.62],
            useFixedBase=True,
        )

        self.cube_id = p.loadURDF("cube_small.urdf", basePosition=self.cube_spawn)
        p.changeDynamics(
            self.cube_id,
            -1,
            lateralFriction=1.0,
            rollingFriction=0.01,
            spinningFriction=0.01,
            restitution=0.0,
            linearDamping=0.05,
            angularDamping=0.05,
        )



        for j, q in zip(self.PANDA_ARM_JOINTS, self.PANDA_HOME):
            p.resetJointState(self.panda_id, j, q)

        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=55,
            cameraPitch=-35,
            cameraTargetPosition=[0.55, 0.0, 0.45],
        )

    def step(self):
        p.stepSimulation()

        if self.cube_id is not None:
            pos, orn = p.getBasePositionAndOrientation(self.cube_id)
            if pos[2] < self.cube_min_z:
                p.resetBasePositionAndOrientation(self.cube_id, self.cube_spawn, orn)
                p.resetBaseVelocity(self.cube_id, [0, 0, 0], [0, 0, 0])

    def get_table_z(self) -> float:
        return float(self.table_z)

    def get_cube_pose(self):
        if self.cube_id is None:
            return None
        pos, orn = p.getBasePositionAndOrientation(self.cube_id)
        return list(pos), list(orn)

    def reset_cube(self, pos=None):
        if self.cube_id is None:
            return
        if pos is None:
            pos = self.cube_spawn
        _, orn = p.getBasePositionAndOrientation(self.cube_id)
        p.resetBasePositionAndOrientation(self.cube_id, pos, orn)
        p.resetBaseVelocity(self.cube_id, [0, 0, 0], [0, 0, 0])

    def disconnect(self):
        if p.isConnected():
            p.disconnect()