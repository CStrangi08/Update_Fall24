from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_CYLINDER, PRIMITIVE_SPHERE
from cwruxr_sdk.built_in_assets import TRANSPARENT_BLACK, LIT_RED
import time
import numpy as np
import random

# Define initial conditions for the particle
pos1 = np.array([0.0, 0.25, 0.1])  # Initial position
vel1 = np.array([random.uniform(-3, 3) for _ in range(3)])   # random Initial velocity
print(f"Initial Random Velocity: ({vel1[0]:.4f}, {vel1[1]:.4f}, {vel1[2]:.4f})") #Tell me the random velocity
radius_particle = 0.05              # Radius of the particle
dt = 0.01                           # Time step
end_time = 5.0                      # Total simulation time

# Define the cylinder parameters
cylinder_radius = 0.5  # Radius of the cylinder (in x-z plane)
cylinder_height = 2    # Height of the cylinder (along the y-axis)

t = np.arange(0, end_time, dt)  # Time array
positions1 = [pos1.copy()]      # List to store positions of the particle over time

# Function to reflect velocity off a surface given the normal vector
def reflect_velocity(vel, normal):
    return vel - 2 * np.dot(vel, normal) * normal

# Simulation loop
for i in range(len(t)):
    # Update position
    pos1 += vel1 * dt

    # Check for collisions with the cylindrical walls (x-z plane)
    distance_from_center = np.sqrt(pos1[0]**2 + pos1[1]**2)
    if distance_from_center >= (cylinder_radius - radius_particle):
        # Calculate the normal vector at the point of collision
        normal = np.array([pos1[0],pos1[1],0]) / distance_from_center
        # Reflect velocity based on the normal vector
        vel1 = reflect_velocity(vel1, normal)

    # Check for collisions with the top and bottom of the cylinder (y direction)
    if pos1[2] >= cylinder_height - radius_particle or pos1[2] <= radius_particle:
        vel1[2] = -vel1[2]  # Reverse velocity in z direction

    # Store the position after update
    positions1.append(pos1.copy())

    # Print the current time and position with formatted output
    print(f"Time: {t[i]:.2f} s, Position 1: ({positions1[-1][0]:.4f}, {positions1[-1][1]:.4f}, {positions1[-1][2]:.4f})")

# Convert positions to a list of lists for easier viewing
positions1 = [p.tolist() for p in positions1]

# Initialize the client
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "BLHD"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Post objects to API
for i in range(len(t)):
    P1 = PrimitiveMessage(
        id="particle1",
        source=PRIMITIVE_SPHERE,
        materialID=LIT_RED,
        pose=Pose(
            position=Vector3(float(positions1[i][0]), float(positions1[i][1]), float(positions1[i][2])),
            rotation=Quaternion(0, 0, -1, 0),
            scale=Vector3(.1, .1, .1)
        ),
        isManipulationOn=False,
    )
    
    cylinder = PrimitiveMessage(
        id="cylinder1",
        source=PRIMITIVE_CYLINDER,
        materialID=TRANSPARENT_BLACK,
        pose=Pose(
            position=Vector3(0, 0, cylinder_height/2),
            rotation=Quaternion(1, 0, 0, 1),
            scale=Vector3(1, 1, 1)
        ),
        isManipulationOn=False,
    )   

    time.sleep(.002)
    createResponse = cwruxrClient.PostObjectBulk([P1, cylinder])

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects()



