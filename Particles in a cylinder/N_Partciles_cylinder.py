from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_CYLINDER
from cwruxr_sdk.built_in_assets import LIT_BLUE, TRANSPARENT_BLACK
import time
import numpy as np
import random

# Define initial conditions for particles
num_particles = 100
cylinder_radius = 1.5 # Radius of the cylinder (in x-z plane)
cylinder_height = 15  # Height of the cylinder (along the y-axis)

# Random initial positions within the cylinder
positions = np.array([[random.uniform(-cylinder_radius, cylinder_radius), 
                       random.uniform(-cylinder_radius, cylinder_radius),
                       random.uniform(0.2, 1.2), ] for _ in range(num_particles)])

# Random initial velocities for particles
velocities = np.array([[random.uniform(-2.0, 2.0), random.uniform(-2.0, 2.0), random.uniform(3.5, 4.0)] for _ in range(num_particles)])  
masses = np.ones(num_particles)  # Assume equal masses for all particles
radius_particle = 0.05  # Radius of each particle

# Ensure initial positions are within the cylindrical boundary
for i in range(num_particles):
    while np.sqrt(positions[i][0]**2 + positions[i][1]**2) >= (cylinder_radius - radius_particle):
        # Regenerate x and y components if outside cylinder boundary
        positions[i][0] = random.uniform(-cylinder_radius, cylinder_radius)
        positions[i][1] = random.uniform(-cylinder_radius, cylinder_radius)

# Function to detect collision between two particles
def detect_collision(pos1, pos2, radius):
    return np.linalg.norm(pos1 - pos2) < 2 * radius

# Function to handle elastic collision between two particles
def elastic_collision(pos1, vel1, pos2, vel2, mass1, mass2):
    normal = (pos2 - pos1) / np.linalg.norm(pos2 - pos1)
    relative_velocity = vel2 - vel1
    speed = np.dot(relative_velocity, normal)

    if speed < 0:  # Only react if they are moving towards each other
        # Calculate impulses for x and y velocities
        impulsex = (2 * speed * mass2) / (mass1 + mass2)
        impulsez = (2 * speed * mass2) / (mass1 + mass2)

        # Update only x and y velocities
        vel1[0] += impulsex * normal[0]
        vel1[1] += impulsez * normal[1]

        vel2[0] -= impulsex * normal[0]
        vel2[1] -= impulsez * normal[1]
    return vel1, vel2


# Simulation parameters
dt = 0.01  # Time step
end_time = 25.0  # Total simulation time
t = np.arange(0, end_time, dt)

# Initialize a list to store positions at each time step
positions_list = []

# Function to reflect velocity off a surface given the normal vector
def reflect_velocity(vel, normal):
    return vel - 2 * np.dot(vel, normal) * normal

# Simulation loop
for i in range(len(t)):
    # Update positions for all particles
    for j in range(num_particles):
        positions[j] += velocities[j] * dt

        # Check for collisions with the cylindrical walls (x-z plane)
        distance_from_center = np.sqrt(positions[j][0]**2 + positions[j][1]**2)
        if distance_from_center >= (cylinder_radius - radius_particle):
            # Calculate the normal vector at the point of collision
            normal = np.array([positions[j][0], positions[j][1],0]) / distance_from_center
            # Reflect velocity based on the normal vector
            velocities[j] = reflect_velocity(velocities[j], normal)

        # Check for collisions with the top and bottom of the cylinder (y direction)
        if positions[j][2] >= cylinder_height - radius_particle or positions[j][2] <= radius_particle:
            velocities[j][2] = -velocities[j][2]  # Reverse velocity in y direction
        
        # Check for collisions between particles
    
    for p1 in range(num_particles):
        for p2 in range(p1 + 1, num_particles):
            if detect_collision(positions[p1], positions[p2], radius_particle):
                velocities[p1], velocities[p2] = elastic_collision(positions[p1], velocities[p1], positions[p2], velocities[p2], masses[p1], masses[p2])

    # Store copies of positions at each time step
    positions_list.append([pos.copy() for pos in positions])

    # Print the current time and positions for all particles
    print(f"Time: {t[i]:.2f} s")
    for j in range(num_particles):
        print(f"  Position {j + 1}: ({positions[j][0]:.4f}, {positions[j][1]:.4f}, {positions[j][2]:.4f})")
    print()  # Add a newline for better readability

positions = [[] for _ in range(num_particles)]  # Create a list for N particles

# Update your simulation code to fill positions
for i in range(len(t)):
    for j in range(num_particles):
        # Update positions for each particle
        positions[j].append([positions_list[i][j][0], positions_list[i][j][1], positions_list[i][j][2]])  # Ensure this reflects your logic


# Initialize the client
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "JSTK"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Post objects to API
for i in range(len(t)):
    particles = []  # List to store particle messages
    
    for j in range(num_particles):  # Create 10 particles
        particle = PrimitiveMessage(
            id=f"particle{j + 1}",  # Unique ID for each particle
            source=PRIMITIVE_SPHERE,
            materialID= LIT_BLUE,
            pose=Pose(
                position=Vector3(float(positions[j][i][0]), float(positions[j][i][1]), float(positions[j][i][2])),
                rotation=Quaternion(0, 0, -1, 0),
                scale=Vector3(.1, .1, .1)
            ),
            isManipulationOn=False,
        )
        particles.append(particle)  # Add particle to the list

    # Create a cylindrical boundary for visualization
    cylinder = PrimitiveMessage(
        id="cylinder1",
        source=PRIMITIVE_CYLINDER,
        materialID=TRANSPARENT_BLACK,
        pose=Pose(
            position=Vector3(0, 0, cylinder_height/2),
            rotation=Quaternion(1, 0, 0, 1),
            scale=Vector3(cylinder_radius * 2, cylinder_height / 2, cylinder_radius * 2)  # Scale appropriately
        ),
        isManipulationOn=False,
    )

    # Post particles and the cylinder
    time.sleep(dt)
    createResponse = cwruxrClient.PostObjectBulk(particles)
    createResponse = cwruxrClient.PostObject(cylinder)

input("Press Enter to Delete")
deleteResponse = cwruxrClient.DeleteAllObjects()