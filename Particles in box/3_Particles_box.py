from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_PLANE
from cwruxr_sdk.built_in_assets import LIT_BLUE, LIT_RED,LIT_GREEN
import time
import numpy as np

# Define initial conditions for particle 1 (colliding with walls)
pos1 = np.array([0.0, 0.8, 0.0])  # Initial position
vel1 = np.array([3.0, -1.8, 2.0])  # Initial velocity
mass1 = 1.0                        # Mass of the particle
radius = 0.05                      # Radius of the particle

# Define initial conditions for particle 2 (colliding with walls)
pos2 = np.array([0.5, 0.2, 0.5])   # Initial position
vel2 = np.array([-2.0, 1.0, -1.0])  # Initial velocity
mass2 = 1.0                        # Mass of the second particle

# Define initial conditions for particle 3 (colliding with walls)
pos3 = np.array([0.2, 0.5, 0.2])  # Initial position
vel3 = np.array([1.0, -2.0, 1.5])     # Initial velocity
mass3 = 1.0                           # Mass of the third particle

# Define the size of the two boxes
Box1_size = np.array([2.5, 1.0, 1.0])  # Size of the first box
Box2_size = np.array([-2.5, 0.0, -1.0])  # Size of the second box

positions1 = [pos1.copy()]  # List to store positions of particle 1
positions2 = [pos2.copy()]  # List to store positions of particle 2
positions3 = [pos3.copy()]  # List to store positions of particle 3

# Function to detect collision between two particles
def detect_collision(pos1, pos2, radius):
    return np.linalg.norm(pos1 - pos2) < 2 * radius

# Function to handle elastic collision between two particles
def elastic_collision(pos1, vel1, pos2, vel2, mass1, mass2):
    normal = (pos2 - pos1) / np.linalg.norm(pos2 - pos1)
    relative_velocity = vel2 - vel1
    speed = np.dot(relative_velocity, normal)
    if speed < 0:
        impulse = 2 * speed / (mass1 + mass2)
        vel1 += impulse * mass2 * normal
        vel2 -= impulse * mass1 * normal
    return vel1, vel2

# Simulation loop
dt = 0.01  # Time step
end_time = 20.0  # Total simulation time
t = np.arange(0, end_time, dt)

for i in range(len(t)):
    # Update positions for all particles
    pos1 += vel1 * dt
    pos2 += vel2 * dt
    pos3 += vel3 * dt

    # Check for collision with walls for particle 1 (Box 1)
    for dim in range(3):
        if pos1[dim] >= Box1_size[dim] - radius or pos1[dim] <= Box2_size[dim] + radius:
            vel1[dim] = -vel1[dim]  # Reverse velocity in the corresponding direction

    # Check for collision with walls for particle 2 (Box 1)
    for dim in range(3):
        if pos2[dim] >= Box1_size[dim] - radius or pos2[dim] <= Box2_size[dim] + radius:
            vel2[dim] = -vel2[dim]  # Reverse velocity in the corresponding direction

    # Check for collision with walls for particle 3 (Box 1)
    for dim in range(3):
        if pos3[dim] >= Box1_size[dim] - radius or pos3[dim] <= Box2_size[dim] + radius:
            vel3[dim] = -vel3[dim]  # Reverse velocity in the corresponding direction

    # Check for collision between particles
    collided = []
    
    if detect_collision(pos1, pos2, radius):
        collided.append((pos1, vel1, pos2, vel2, mass1, mass2))
    
    if detect_collision(pos1, pos3, radius):
        collided.append((pos1, vel1, pos3, vel3, mass1, mass3))
    
    if detect_collision(pos2, pos3, radius):
        collided.append((pos2, vel2, pos3, vel3, mass2, mass3))

    # Resolve collisions
    for p1, v1, p2, v2, m1, m2 in collided:
        v1, v2 = elastic_collision(p1, v1, p2, v2, m1, m2)

    # Store positions
    positions1.append(pos1.copy())
    positions2.append(pos2.copy())
    positions3.append(pos3.copy())

    # Print the current time and positions
    print(f"Time: {t[i]:.2f} s, Position 1: ({positions1[-1][0]:.4f}, {positions1[-1][1]:.4f}, {positions1[-1][2]:.4f}), "
          f"Position 2: ({positions2[-1][0]:.4f}, {positions2[-1][1]:.4f}, {positions2[-1][2]:.4f}), "
          f"Position 3: ({positions3[-1][0]:.4f}, {positions3[-1][1]:.4f}, {positions3[-1][2]:.4f})")

# Convert positions to a list of lists for easier viewing
positions1 = [p.tolist() for p in positions1]
positions2 = [p.tolist() for p in positions2]
positions3 = [p.tolist() for p in positions3]

# Initialize the client
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "NWHN"
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

    P3 = PrimitiveMessage(
        id="particle3",
        source=PRIMITIVE_SPHERE,
        materialID=LIT_GREEN,
        pose=Pose(
            position=Vector3(float(positions3[i][0]), float(positions3[i][1]), float(positions3[i][2])),
            rotation=Quaternion(0, 0, -1, 0),
            scale=Vector3(.1, .1, .1)
        ),
        isManipulationOn=False,
    )

    time.sleep(.002)
    createResponse = cwruxrClient.PostObjectBulk([P1, P2,P3])
    

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects
