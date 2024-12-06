#Two Particle Collision, Elastic, Very shaky on the graphics (try other computer)

from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE
from cwruxr_sdk.built_in_assets import LIT_BLUE, LIT_RED
import time
import numpy as np

# Define initial conditions
pos1 = np.array([0.0, 1.0, 0.0])
vel1 = np.array([0.8, 0.1, 0.0])
pos2 = np.array([1.0, 1.0, 0.0])
vel2 = np.array([-20, 0.0, 0.2])
mass1 = mass2 = 1.0  # Assume equal masses
radius = 0.05  # Radius of the particles
dt = 0.0001  # Time step
time1 = 0.0
end_time = 2.0

# Function to detect collision
def detect_collision(pos1, pos2, radius):
    return np.linalg.norm(pos1 - pos2) < 2 * radius

# Function to handle elastic collision
def elastic_collision(pos1, vel1, pos2, vel2, mass1, mass2):
    normal = (pos2 - pos1) / np.linalg.norm(pos2 - pos1)
    relative_velocity = vel2 - vel1
    speed = np.dot(relative_velocity, normal)
    if speed < 0:
        impulse = 2 * speed / (mass1 + mass2)
        vel1 += impulse * mass2 * normal
        vel2 -= impulse * mass1 * normal
    return vel1, vel2

# Initialize lists to store positions
positions1 = [pos1.copy()]
positions2 = [pos2.copy()]

# Simulation loop
t = np.arange(0, end_time, dt)

for i in range(len(t)):
    # Update positions
    pos1 += vel1 * dt
    pos2 += vel2 * dt

    # Check for collision
    if detect_collision(pos1, pos2, radius):
        vel1, vel2 = elastic_collision(pos1, vel1, pos2, vel2, mass1, mass2)

    # Store positions
    positions1.append(pos1.copy())
    positions2.append(pos2.copy())

    # Print current position as floats
    print(f"Time: {t[i]:.2f} s, Position 1: ({positions1[-1][0]:.4f}, {positions1[-1][1]:.4f}, {positions1[-1][2]:.4f}), "
          f"Position 2: ({positions2[-1][0]:.4f}, {positions2[-1][1]:.4f}, {positions2[-1][2]:.4f})")


positions1 = [p.tolist() for p in positions1]
positions2 = [p.tolist() for p in positions2]

# Initialize the client
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "SDBV"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Post objects to API
for i in range(len(t)):
    P1 = PrimitiveMessage(
        id="particle1",
        source=PRIMITIVE_SPHERE,
        materialID=LIT_BLUE,
        pose=Pose(
            position=Vector3(float(positions1[i][0]), float(positions1[i][1]),float(positions1[i][2])),
            rotation=Quaternion(0, 0, -1, 0),
            scale=Vector3(.1, .1, .1)
        ),
        isManipulationOn=False,
    )

    P2 = PrimitiveMessage(
        id="particle2",
        source=PRIMITIVE_SPHERE,
        materialID=LIT_RED,
        pose=Pose(
            position=Vector3(float(positions2[i][0]), float(positions2[i][1]), float(positions2[i][2])),
            rotation=Quaternion(0, 0, -1, 0),
            scale=Vector3(.1, .1, .1)
        ),
        isManipulationOn=False,
    )
    time.sleep(dt)
    createResponse = cwruxrClient.PostObjectBulk([P1, P2])
    

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects







