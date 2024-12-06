import cupy as cp
import numpy as np
import time
import random
from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_CYLINDER
from cwruxr_sdk.built_in_assets import LIT_BLUE, TRANSPARENT_BLACK
import threading

# Particle simulation parameters
num_particles = 100
cylinder_radius = 1.5  # Radius of the cylinder (in x-z plane)
cylinder_height = 15  # Height of the cylinder (along the y-axis)
positions = cp.array([[random.uniform(-cylinder_radius+.1, cylinder_radius-.1), 
                       random.uniform(-cylinder_radius+.1, cylinder_radius-.1),
                       random.uniform(0.2, 1.2)] for _ in range(num_particles)])
velocities = cp.array([[random.uniform(-2.0*5, 2.0*5), random.uniform(-2.0*5, 2.0*5), random.uniform(3.5*5, 4.0*5)] for _ in range(num_particles)])  
masses = cp.ones(num_particles)  # Assume equal masses
radius_particle = 0.05  # Radius of each particle

# Ensure initial positions are within the cylindrical boundary
for i in range(num_particles):
    while cp.sqrt(positions[i][0]**2 + positions[i][1]**2) >= (cylinder_radius - radius_particle):
        positions[i][0] = random.uniform(-cylinder_radius, cylinder_radius)
        positions[i][1] = random.uniform(-cylinder_radius, cylinder_radius)

# Simulation parameters
dt = 0.01  # Time step

# Initialize the client for API communication
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "TMBF"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Cylinder object definition
cylinder = PrimitiveMessage(
    id="cylinder1",
    source=PRIMITIVE_CYLINDER,
    materialID=TRANSPARENT_BLACK,
    pose=Pose(
        position=Vector3(0, 0, cylinder_height / 2),
        rotation=Quaternion(1, 0, 0, 1),
        scale=Vector3(cylinder_radius * 2, cylinder_height / 2, cylinder_radius * 2)  # Scale appropriately
    ),
    isManipulationOn=False,
)

# Post the cylinder object initially
createResponse = cwruxrClient.PostObject(cylinder)

# Function to update positions and check for collisions
def update_positions_and_check_walls(positions, velocities, radius_particle, cylinder_radius, cylinder_height):
    positions += velocities * dt

    # Check for collisions with the cylindrical walls (x-y plane)
    distance_from_center = cp.sqrt(positions[:, 0]**2 + positions[:, 1]**2)
    out_of_bounds_cylinder = distance_from_center >= (cylinder_radius - radius_particle)

    # Reflect velocities for out-of-bounds particles
    velocities[out_of_bounds_cylinder, 0] *= -1
    velocities[out_of_bounds_cylinder, 1] *= -1

    # Check for collisions with the top and bottom of the cylinder (z direction)
    top_bottom_mask = (positions[:, 2] >= cylinder_height - radius_particle) | (positions[:, 2] <= radius_particle)
    
    for i in range(len(positions)):
        if top_bottom_mask[i]:
            # Delete the particle from the CWRUXR environment
            delete_response = cwruxrClient.DeleteObject(f"particle{i + 1}")

            # Respawn at a random position within the cylinder's base (x, y random, z set to a low value)
            new_position = cp.array([random.uniform(-cylinder_radius+.1, cylinder_radius-.1), 
                       random.uniform(-cylinder_radius+.1, cylinder_radius-.1),
                       random.uniform(0.2, 1.2)])  # Respawn near the bottom of the cylinder
            positions[i] = new_position

            # Reset velocity
            velocities[i] = cp.array([random.uniform(-2.0, 2.0), 
                                      random.uniform(-2.0, 2.0), 
                                      random.uniform(3.5, 4.0)])  # Reset velocity

            # Repost the particle in the CWRUXR environment
            particle = PrimitiveMessage(
                id=f"particle{i + 1}",
                source=PRIMITIVE_SPHERE,
                materialID=LIT_BLUE,
                pose=Pose(
                    position=Vector3(float(positions[i][0]), float(positions[i][1]), float(positions[i][2])),
                    rotation=Quaternion(0, 0, -1, 0),
                    scale=Vector3(.1, .1, .1)
                ),
                isManipulationOn=True,
            )
            create_response = cwruxrClient.PostObject(particle)


# Function for handling particle-particle collisions
def handle_collisions(positions, velocities, masses, radius_particle):
    dist_matrix = cp.linalg.norm(positions[:, cp.newaxis] - positions[cp.newaxis, :], axis=2)
    collision_mask = (dist_matrix < 2 * radius_particle) & (cp.triu(cp.ones((num_particles, num_particles), dtype=bool), k=1))

    if cp.any(collision_mask):
        p1, p2 = cp.where(collision_mask)
        for i in range(len(p1)):
            pos1, pos2 = positions[p1[i]], positions[p2[i]]
            vel1, vel2 = velocities[p1[i]], velocities[p2[i]]
            normal = (pos2 - pos1) / cp.linalg.norm(pos2 - pos1)
            relative_velocity = vel2 - vel1
            speed = cp.dot(relative_velocity, normal)

            if speed < 0:
                impulse = 2 * speed / (masses[p1[i]] + masses[p2[i]])
                velocities[p1[i]] += impulse * masses[p2[i]] * normal
                velocities[p2[i]] -= impulse * masses[p1[i]] * normal

# Flag to stop simulation
stop_simulation = False

# Thread for getting input
def input_thread():
    global stop_simulation, cylinder_posted
    while True:
        user_input = input("Enter 'p' to post the cylinder, 'd' to delete it:")
        if user_input == 'p':
            if not cylinder_posted:
                print("Posting the cylinder.")
                createResponse = cwruxrClient.PostObject(cylinder)
                cylinder_posted = True
        elif user_input == 'd':
            if cylinder_posted:
                print("Deleting the cylinder.")
                deleteResponse = cwruxrClient.DeleteObject("cylinder1")
                cylinder_posted = False
       

# Start the input thread
thread = threading.Thread(target=input_thread)
thread.start()

# Infinite loop for real-time updates
cylinder_posted = True  # Track whether the cylinder is posted or not

try:
    while not stop_simulation:
        # --- Step 1: Update positions and handle collisions ---
        update_positions_and_check_walls(positions, velocities, radius_particle, cylinder_radius, cylinder_height)
        handle_collisions(positions, velocities, masses, radius_particle)

        # --- Step 2: Send real-time updates to CWRUXR API ---
        current_positions = cp.asnumpy(positions.copy())  # Convert positions to NumPy for posting

        particles = []
        for j in range(num_particles):
            particle = PrimitiveMessage(
                id=f"particle{j + 1}",
                source=PRIMITIVE_SPHERE,
                materialID=LIT_BLUE,
                pose=Pose(
                    position=Vector3(float(current_positions[j][0]), float(current_positions[j][1]), float(current_positions[j][2])),
                    rotation=Quaternion(0, 0, -1, 0),
                    scale=Vector3(.1, .1, .1)
                ),
                isManipulationOn=True,
            )
            particles.append(particle)

        # Post only the updated particles to the API
        createResponse = cwruxrClient.PostObjectBulk(particles)

        # Delay for real-time effect (based on time step)
        time.sleep(dt)

except KeyboardInterrupt:
    print("Simulation stopped by user.")

# Wait for input thread to finish
thread.join()
deleteResponse = cwruxrClient.DeleteAllObjects()




