import cupy as cp
import numpy as np
import time
import random
from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_CUBE
from cwruxr_sdk.built_in_assets import LIT_BLUE, TRANSPARENT_BLACK
import threading

# Particle simulation parameters
num_particles = 10
box_length = 10  # Length of the rectangular box along the x-axis
box_width = 2.0  # Width of the rectangular box along the y-axis
box_height = 1.0  # Height of the rectangular box along the z-axis
radius_particle = 0.05  # Radius of each particle

# Initialize particle positions within the rectangular box
positions = cp.random.uniform(-0.75, 0.75, (num_particles, 3))

# Initialize velocities
velocities = cp.array([[random.uniform(-2.0, 2.0), random.uniform(-2.0, 2.0), random.uniform(-2.0, 2.0)] for _ in range(num_particles)])

masses = cp.ones(num_particles)  # Assume equal masses

# Simulation parameters
dt = 0.01  # Time step

# Initialize the client for API communication
ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "DVST"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Box object definition (replace the cylinder with a cube/rectangular box)
box = PrimitiveMessage(
    id="box1",
    source=PRIMITIVE_CUBE,
    materialID=TRANSPARENT_BLACK,
    pose=Pose(
        position=Vector3(0, 0, 0),
        rotation=Quaternion(1, 0, 0, 1),
        scale=Vector3(box_length, box_width, box_height *2)  # Scale appropriately for rectangular box
    ),
    isManipulationOn=False,
)

# Post the box object initially
createResponse = cwruxrClient.PostObject(box)

# Function to update positions and check for collisions with box walls
def update_positions_and_check_walls(positions, velocities, radius_particle, box_length, box_width, box_height):
    positions += velocities * dt

    # Check for collisions with the walls of the rectangular box along x, y, and z axes
    out_of_bounds_x = (positions[:, 0] >= (box_length / 2 - radius_particle)) | (positions[:, 0] <= (-box_length / 2 + radius_particle))
    out_of_bounds_y = (positions[:, 1] >= (box_height - radius_particle)) | (positions[:, 1] <= - box_height + radius_particle)
    out_of_bounds_z = (positions[:, 2] >= (box_width / 2 - radius_particle)) | (positions[:, 2] <= (-box_width / 2 + radius_particle))

    # Reflect velocities for particles hitting the walls
    velocities[out_of_bounds_x, 0] *= -1
    velocities[out_of_bounds_y, 1] *= -1
    velocities[out_of_bounds_z, 2] *= -1

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
    global stop_simulation, box_posted
    while True:
        user_input = input("Enter 'p' to post the box, 'd' to delete it:")
        if user_input == 'p':
            if not box_posted:
                print("Posting the box.")
                createResponse = cwruxrClient.PostObject(box)
                box_posted = True
        elif user_input == 'd':
            if box_posted:
                print("Deleting the box.")
                deleteResponse = cwruxrClient.DeleteObject("box1")
                box_posted = False

# Start the input thread
thread = threading.Thread(target=input_thread)
thread.start()

# Infinite loop for real-time updates
box_posted = True  # Track whether the box is posted or not

try:
    while not stop_simulation:
        # --- Step 1: Update positions and handle collisions ---
        update_positions_and_check_walls(positions, velocities, radius_particle, box_length, box_width, box_height)
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

