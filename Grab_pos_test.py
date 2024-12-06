import cupy as cp
import time
from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE
from cwruxr_sdk.built_in_assets import LIT_BLUE
import threading

# Constants
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "HWWP"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
DT = 0.01  # Time step for updates
NUM_PARTICLES = 2

# Initialize client
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Initial positions and velocities
positions = cp.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])  # Positions for 2 particles
velocities = cp.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])  # Velocities for 2 particles

# Stop simulation flag
stop_simulation = False


# Function to monitor and log object positions when manipulation changes
def monitor_object_position():
    last_grabbed_status = [False] * NUM_PARTICLES  # Tracks the previous state for each particle

    while not stop_simulation:
        for i in range(NUM_PARTICLES):
            obj_info = cwruxrClient.GetObject(f"particle{i + 1}")
            if obj_info is not None:
                is_grabbed = obj_info.get("isManipulating", False)
                current_position = obj_info["pose"]["position"]
                positions[i] = cp.array([current_position['x'], current_position['y'], current_position['z']])
                
                if last_grabbed_status[i] and not is_grabbed:  # Transition from grabbed to not grabbed
                    print(f"Particle {i + 1} released at position: {positions[i][0]}, {positions[i][1]}, {positions[i][2]}")

                # Update the last grabbed status
                last_grabbed_status[i] = is_grabbed

        time.sleep(.001)  # Avoid tight looping


# Function to update positions based on velocity
def update_positions_and_check_walls(positions, velocities):
    positions += velocities * DT
    return positions


# Post particles initially
particles = []
for i in range(NUM_PARTICLES):
    particle_id = f"particle{i + 1}"
    particle = PrimitiveMessage(
        id=particle_id,
        source=PRIMITIVE_SPHERE,
        materialID=LIT_BLUE,
        pose=Pose(
            position=Vector3(float(positions[i][0]), float(positions[i][1]), float(positions[i][2])),
            rotation=Quaternion(0, 0, -1, 0),
            scale=Vector3(0.1, 0.1, 0.1)
        ),
        isManipulationOn=True
    )
    cwruxrClient.PostObject(particle)
    particles.append(particle)

# Start the monitoring thread
monitor_thread = threading.Thread(target=monitor_object_position, daemon=True)
monitor_thread.start()

# Simulation loop
try:
    while not stop_simulation:
        start_time = time.time()

        # Update positions
        positions = update_positions_and_check_walls(positions, velocities)

        # Create and update particles in CWRUXR API
        particles = [
            PrimitiveMessage(
                id=f"particle{j + 1}",
                source=PRIMITIVE_SPHERE,
                materialID=LIT_BLUE,
                pose=Pose(
                    position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                    rotation=Quaternion(0, 0, -1, 0),
                    scale=Vector3(0.1, 0.1, 0.1)
                ),
                isManipulationOn=True
            ) for j, pos in enumerate(cp.asnumpy(positions))
        ]

        cwruxrClient.PostObjectBulk(particles)

        # End simulation step
        end_time = time.time()
        calc_duration = end_time - start_time
        # print(f"Calculation time for this iteration: {calc_duration:.6f} seconds")

        time.sleep(DT)

except KeyboardInterrupt:
    print("Simulation stopped by user.")

finally:
    stop_simulation = True  # Ensure monitoring thread exits
    monitor_thread.join()
    cwruxrClient.DeleteAllObjects()
    print("All objects deleted. Cleanup complete.")




