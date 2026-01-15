import pybullet as p
import pybullet_data

class World:
    def __init__(self):
        self.PANDA_ARM_JOINTS = [0, 1, 2, 3, 4, 5, 6]
        self.PANDA_HOME = [0.0, -0.4, 0.0, -2.2, 0.0, 2.0, 0.8]
        self.panda_id = None
        self.cube_id = None
        self.table_id = None
        self.table_pos = [0.6, 0.0, 0.0]
        self.cube_spawn = [0.55, 0.0, 0.70]
        self.cube_min_z = 0.58

    def setup(self):
        p.connect(p.GUI)
        # gpu optimizations for intel uhd
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        
        # load ground and table
        p.loadURDF("plane.urdf")
        self.table_id = p.loadURDF("table/table.urdf", basePosition=self.table_pos, useFixedBase=True)
        self.table_z = p.getAABB(self.table_id)[1][2]

        # load robot and target
        self.panda_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0.0, 0.0, 0.62], useFixedBase=True)
        self.cube_id = p.loadURDF("cube_small.urdf", basePosition=self.cube_spawn)
        p.changeDynamics(self.cube_id, -1, lateralFriction=2.0)

        for j, q in zip(self.PANDA_ARM_JOINTS, self.PANDA_HOME):
            p.resetJointState(self.panda_id, j, q)

        # camera position
        self.camera_yaw = 10
        p.resetDebugVisualizerCamera(
            cameraDistance=1,
            cameraYaw=self.camera_yaw,
            cameraPitch=-40,
            cameraTargetPosition=[0.65, 0, 0.75]
        )

    def step(self):
        p.stepSimulation()

    def disconnect(self):
        if p.isConnected(): p.disconnect()