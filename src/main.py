import time
import pybullet as p
import pybullet_data

# Constants for robot configuration
PANDA_ARM_JOINTS = [0, 1, 2, 3, 4, 5, 6]
PANDA_HOME = [0.0, -0.4, 0.0, -2.2, 0.0, 2.0, 0.8]

def setup_world():
    # Initializes environment
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    # Load environment assets
    p.loadURDF("plane.urdf")
    p.loadURDF(
        "table/table.urdf",
        basePosition=[0.6, 0.0, 0.0],
        useFixedBase=True,
    )

    # Load robot
    panda_id = p.loadURDF(
        "franka_panda/panda.urdf",
        basePosition=[0.0, 0.0, 0.62],
        useFixedBase=True,
    )

    # Load target object
    cube_id = p.loadURDF("cube_small.urdf", basePosition=[0.55, 0.0, 0.68])

    # Set robot home configuration
    for j, q in zip(PANDA_ARM_JOINTS, PANDA_HOME):
        p.resetJointState(panda_id, j, q)

    # Adjust camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=55,
        cameraPitch=-35,
        cameraTargetPosition=[0.55, 0.0, 0.45],
    )

    return panda_id, cube_id


def main():
    cid = p.connect(p.GUI)
    if cid < 0:
        raise RuntimeError("PyBullet GUI connection failed")

    try:
        setup_world()
        p.setRealTimeSimulation(0)

        # Simulation loop
        for _ in range(2400):
            if not p.isConnected():
                break
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()