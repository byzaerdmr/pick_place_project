import pybullet as p
import pybullet_data

class World:
    def __init__(self):
        # Constants for robot configuration
        self.PANDA_ARM_JOINTS = [0, 1, 2, 3, 4, 5, 6]
        self.PANDA_HOME = [0.0, -0.4, 0.0, -2.2, 0.0, 2.0, 0.8]
        self.panda_id = None
        self.cube_id = None

    def setup(self):
        # Initializes environment
        cid = p.connect(p.GUI)
        if cid < 0:
            raise RuntimeError("PyBullet GUI connection failed")
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        # Physics quality (anti-tunneling)
        p.setTimeStep(1.0 / 240.0)
        p.setPhysicsEngineParameter(
            fixedTimeStep=1.0 / 240.0,
            numSolverIterations=150,
            numSubSteps=4,
            enableConeFriction=1,
            contactBreakingThreshold=0.001,
        )

        # Disable real-time simulation (we will step manually)
        p.setRealTimeSimulation(0)

        # Load environment assets
        p.loadURDF("plane.urdf")
        p.loadURDF(
            "table/table.urdf",
            basePosition=[0.6, 0.0, 0.0],
            useFixedBase=True,
        )

        # Load robot
        self.panda_id = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[0.0, 0.0, 0.62],
            useFixedBase=True,
        )

        # Load target object
        self.cube_id = p.loadURDF("cube_small.urdf", basePosition=[0.55, 0.0, 0.70])

        # Set robot home configuration
        for j, q in zip(self.PANDA_ARM_JOINTS, self.PANDA_HOME):
            p.resetJointState(self.panda_id, j, q)

        # Adjust camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=55,
            cameraPitch=-35,
            cameraTargetPosition=[0.55, 0.0, 0.45],
        )

    def step(self):
        p.stepSimulation()

    def disconnect(self):
        if p.isConnected():
            p.disconnect()