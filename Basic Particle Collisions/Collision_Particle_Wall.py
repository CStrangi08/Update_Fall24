from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_PLANE
from cwruxr_sdk.built_in_assets import LIT_BLUE, LIT_RED
import time
import numpy as np

# Define initial conditions of the particle
import numpy as np

# Define initial conditions of the particle
pos1 = np.array([-1.0, 1.0, -1.0])  # Initial position
vel1 = np.array([1.5, -1.2, 2.0])   # Initial velocity
mass1 = 1.0                    # Mass of the particle
radius = 0.05                  # Radius of the particle
dt = 0.01                      # Time step
end_time = 3.0                 # Total simulation time

poswall = np.array([0, 1, 0])  # Position of the wall (assume it's flat in the xy plane)

t = np.arange(0, end_time, dt)  # Time array

positions1 = [pos1.copy()]  # List to store positions of the particle over time
wallpos = [poswall.copy()]  # List to store the position of the wall (it remains stationary)

for i in range(len(t)):
    # Update positions
    pos1 += vel1 * dt
    
    # Check for collision with the wall (z = 0 plane), using a tolerance
    if pos1[2] >= poswall[2] - radius:
        vel1[2] = -vel1[2]  # Reverse the velocity in the z direction upon collision
    
    # Store positions
    positions1.append(pos1.copy())
    wallpos.append(poswall.copy())  # Append the wall's position at each step (stationary)
    
    # Print the current time and position with formatted output
    print(f"Time: {t[i]:.2f} s, Position 1: ({positions1[-1][0]:.4f}, {positions1[-1][1]:.4f}, {positions1[-1][2]:.4f})")

# Convert positions to a list of lists for easier viewing
positions1 = [p.tolist() for p in positions1]
wallpos = [p.tolist() for p in wallpos]




# Initialize the client
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "SRPX"
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

    wall = PrimitiveMessage(
        id="wall1",
        source=PRIMITIVE_PLANE,
        materialID=LIT_BLUE,
        pose=Pose(
            position=Vector3(float(wallpos[i][0]), float(wallpos[i][1]),float(wallpos[i][2])),
            rotation=Quaternion(1, 0, 0, -1),
            scale=Vector3(.25, .25, .25)
        ),
        isManipulationOn=False,
    )
    
    
    time.sleep(dt)
    createResponse = cwruxrClient.PostObjectBulk([P1])
    

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects








