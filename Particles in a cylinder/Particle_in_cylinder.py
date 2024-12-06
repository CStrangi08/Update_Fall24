from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_CYLINDER, PRIMITIVE_SPHERE
from cwruxr_sdk.built_in_assets import TRANSPARENT_BLACK, LIT_RED
import time
import numpy as np
import random

# Define initial conditions for the particle
pos1 = np.array([0.2, 0.3, -0.2])  # Initial position
vel1 = np.array([random.uniform(-3, 3) for _ in range(3)])  # Random initial velocity
print(f"Initial Random Velocity: ({vel1[0]:.4f}, {vel1[1]:.4f}, {vel1[2]:.4f})")  # Display the random velocity
radius_particle = 0.05  # Radius of the particle
dt = 0.01  # Time step
end_time = 5.0  # Total simulation time

# Define the cylinder parameters
cylinder_radius = 0.5  # Radius of the cylinder (in x-z plane)
cylinder_height = 2  # Height of the cylinder (along the y-axis)

# Define the hole parameters
hole_center = np.array([0.4, 0.5, 0.0])  # Hole center on the x-z plane
hole_radius = 0.1  # Radius of the hole

t = np.arange(0, end_time, dt)  # Time array
positions1 = [pos1.copy()]  # List to store positions of the particle over time
particle_exited = False     # Flag to indicate if the particle has exited

# Function to reflect velocity off a surface given the normal vector
def reflect_velocity(vel, normal):
    return vel - 2 * np.dot(vel, normal) * normal

# Function to check if the particle is inside the hole area
def is_in_hole(pos, hole_center, hole_radius):
    # Calculate distance from the hole center (ignore y-axis)
    distance_to_hole = np.sqrt((pos[0] - hole_center[0])**2 + (pos[2] - hole_center[2])**2)
    return distance_to_hole <= hole_radius

# Simulation loop
for i in range(len(t)):
    # Update position
    pos1 += vel1 * dt

    # Check if the particle is within cylinder boundary and outside of hole area
    distance_from_center = np.sqrt(pos1[0]**2 + pos1[2]**2)
    if not particle_exited:  # Only apply cylinder constraints if particle has not exited
        if distance_from_center >= (cylinder_radius - radius_particle) and not is_in_hole(pos1, hole_center, hole_radius):
            # Calculate the normal vector at the point of collision
            normal = np.array([pos1[0], 0, pos1[2]]) / distance_from_center
            # Reflect velocity based on the normal vector
            vel1 = reflect_velocity(vel1, normal)
        elif is_in_hole(pos1, hole_center, hole_radius):
            # Allow particle to exit if it enters the hole area
            print(f"Particle exited through the hole at time {t[i]:.2f} s.")
            particle_exited = True  # Set flag to indicate particle has exited the cylinder

    # Check for collisions with the top and bottom of the cylinder (y direction)
    if not particle_exited:  # Only reflect in y-direction if inside the cylinder
        if pos1[1] >= cylinder_height - radius_particle or pos1[1] <= radius_particle:
            vel1[1] = -vel1[1]  # Reverse velocity in y direction

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
    # Particle representation
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

    # Cylinder representation (without the hole for simplicity)
    cylinder = PrimitiveMessage(
        id="cylinder1",
        source=PRIMITIVE_CYLINDER,
        materialID=TRANSPARENT_BLACK,
        pose=Pose(
            position=Vector3(0, 1, 0),
            rotation=Quaternion(0, 0, 0, 1),
            scale=Vector3(1, 1, 1)
        ),
        isManipulationOn=False,
    )

    time.sleep(.002)
    createResponse = cwruxrClient.PostObjectBulk([P1, cylinder])

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects()


