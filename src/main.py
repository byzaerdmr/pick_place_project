import time
from simulation.world import World

def main():
    # Create World instance
    world = World()

    try:
        world.setup()
        
        # Simulation loop
        for _ in range(2400):
            world.step()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        pass
    finally:
        world.disconnect()

if __name__ == "__main__":
    main()