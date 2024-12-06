from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_CUBE
from cwruxr_sdk.built_in_assets import LIT_BLUE, LIT_RED,LIT_GREEN, TRANSPARENT_BLACK
import time
import numpy as np

# Define initial conditions for particles
num_particles = 10
positions = np.random.uniform(-0.5, 0.5, (num_particles, 3))  # Random initial positions within the box
velocities = np.random.uniform(-2.0, 2.0, (num_particles, 3))  # Random initial velocities
masses = np.ones(num_particles)  # Assume equal masses
radius = 0.05  # Radius of each particle

# Define the size of the two boxes
Box1_size = np.array([2.5, 1.0, 1.0])  # Size of the first box
Box2_size = np.array([-2.5, 0.0, -1.0])  # Size of the second box

positions_list = [positions[i].copy() for i in range(num_particles)]  # Store initial positions

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
end_time = 25.0  # Total simulation time
t = np.arange(0, end_time, dt)

# Initialize a list to store positions at each time step
positions_list = []

# Simulation loop
for i in range(len(t)):
    # Update positions for all particles
    for j in range(num_particles):
        positions[j] += velocities[j] * dt
        
        # Check for collision with walls
        for dim in range(3):
            if positions[j][dim] >= Box1_size[dim] - radius or positions[j][dim] <= Box2_size[dim] + radius:
                velocities[j][dim] = -velocities[j][dim]  # Reverse velocity in the corresponding direction

    # Check for collisions between particles
    for p1 in range(num_particles):
        for p2 in range(p1 + 1, num_particles):
            if detect_collision(positions[p1], positions[p2], radius):
                velocities[p1], velocities[p2] = elastic_collision(positions[p1], velocities[p1], positions[p2], velocities[p2], masses[p1], masses[p2])

    # Store positions for each particle
    positions_list.append([pos.copy() for pos in positions])  # Store copies of positions

    # Print the current time and positions for all particles
    print(f"Time: {t[i]:.2f} s")
    for j in range(num_particles):
        print(f"  Position {j + 1}: ({positions[j][0]:.4f}, {positions[j][1]:.4f}, {positions[j][2]:.4f})")
    print()  # Add a newline for better readability


# Initialize positions for 10 particles
positions = [[] for _ in range(num_particles)]  # Create a list for 10 particles

# Update your simulation code to fill positions
for i in range(len(t)):
    for j in range(num_particles):
        # Update positions for each particle
        positions[j].append([positions_list[i][j][0], positions_list[i][j][1], positions_list[i][j][2]])  # Ensure this reflects your logic

# Now initialize the client
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "RDWY"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Post objects to API
for i in range(len(t)):
    particles = []  # List to store particle messages
    
    for j in range(num_particles):  # Create 10 particles
        particle = PrimitiveMessage(
            id=f"particle{j + 1}",  # Unique ID for each particle
            source=PRIMITIVE_SPHERE,
            materialID=LIT_BLUE if j % 3 == 0 else (LIT_RED if j % 3 == 1 else LIT_GREEN),
            pose=Pose(
                position=Vector3(float(positions[j][i][0]), float(positions[j][i][1]), float(positions[j][i][2])),
                rotation=Quaternion(0, 0, -1, 0),
                scale=Vector3(.1, .1, .1)
            ),
            isManipulationOn=False,
        )
        particles.append(particle)  # Add particle to the list

    box = PrimitiveMessage(
        id="box",
        source=PRIMITIVE_CUBE,
        materialID=TRANSPARENT_BLACK,
        pose=Pose(
            position=Vector3(0, 0, 0),
            rotation=Quaternion(0, 0, 0, 1),
            scale=Vector3(5, 2, 2)
        ),
        isManipulationOn=False,
    )   

    time.sleep(dt)
    createResponse = cwruxrClient.PostObjectBulk(particles)
    createResponse = cwruxrClient.PostObject(box)  # Post all particles at once

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects()
