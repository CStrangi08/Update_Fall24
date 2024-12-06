from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_PLANE
from cwruxr_sdk.built_in_assets import LIT_BLUE, LIT_RED
import time
import numpy as np

# Define initial conditions for the particle
pos1 = np.array([0.0, 0.5, 0.0])  # Initial position
vel1 = np.array([3, -1.8, 2.0])   # Initial velocity
radius = 0.05                      # Radius of the particle
dt = 0.01                          # Time step
end_time = 5.0                     # Total simulation time

# Define the box boundaries (walls)
Box_size = np.array([0.5, 1, 0.5])   # Max limits of the box (positive side)
Box2_size = np.array([-0.5, 0, -0.5])  # Min limits of the box (negative side)

t = np.arange(0, end_time, 0.01)  # Time array
positions1 = [pos1.copy()]  # List to store positions of the particle over time

# Simulation loop
for i in range(len(t)):
    # Update position
    pos1 += vel1 * dt
    
    # Check for collisions with the walls and reverse velocity accordingly
    if pos1[0] >= Box_size[0] - radius or pos1[0] <= Box2_size[0] + radius:
        vel1[0] = -vel1[0]  # Reverse velocity in x direction

    if pos1[1] >= Box_size[1] - radius or pos1[1] <= Box2_size[1] + radius:
        vel1[1] = -vel1[1]  # Reverse velocity in y direction

    if pos1[2] >= Box_size[2] - radius or pos1[2] <= Box2_size[2] + radius:
        vel1[2] = -vel1[2]  # Reverse velocity in z direction

    # Store the position after update
    positions1.append(pos1.copy())
    
    # Print the current time and position with formatted output
    print(f"Time: {t[i]:.2f} s, Position 1: ({positions1[-1][0]:.4f}, {positions1[-1][1]:.4f}, {positions1[-1][2]:.4f})")

# Convert positions to a list of lists for easier viewing
positions1 = [p.tolist() for p in positions1]


# Initialize the client
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "WTWW"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Post objects to API
for i in range(len(t)):
    P1 = PrimitiveMessage(
        id="particle1",
        source=PRIMITIVE_SPHERE,
        materialID=LIT_RED,
        pose=Pose(
            position=Vector3(float(positions1[i][0]), float(positions1[i][1]),float(positions1[i][2])),
            rotation=Quaternion(0, 0, -1, 0),
            scale=Vector3(.1, .1, .1)
        ),
        isManipulationOn=False,
    )

    
    
    time.sleep(dt)
    createResponse = cwruxrClient.PostObjectBulk([P1])
    

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects









