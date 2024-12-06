import cupy as cp
import time
from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE,PRIMITIVE_CUBE
from cwruxr_sdk.built_in_assets import LIT_BLUE,TRANSPARENT_BLACK
import threading
import random

box_length = 6.0 #length x length base path
box_height = 1.5 #height = width, logic creates square cross sectional area
box_width = 1.5

ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "HWWP"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Box object definition, fully scalable
box1 = PrimitiveMessage(
    id="box1",
    source=PRIMITIVE_CUBE,
    materialID=TRANSPARENT_BLACK,
    pose=Pose(
        position=Vector3(0, box_height/4,0),
        rotation=Quaternion(1, 0, 0, 1),
        scale=Vector3(box_length, box_width, box_height/2)  
    ),
    isManipulationOn=False,
)

box2 = PrimitiveMessage(
    id="box2",
    source=PRIMITIVE_CUBE,
    materialID=TRANSPARENT_BLACK,
    pose=Pose(
        position=Vector3((box_length/2)-(box_width/2), box_height/4,(-box_length/2)+(box_width/2)),
        rotation=Quaternion(0, 1, 0, 1),
        scale=Vector3(box_length, box_width/2, box_height)  # Scale appropriately for rectangular box
    ),
    isManipulationOn=False,
)

box3 = PrimitiveMessage(
    id="box3",
    source=PRIMITIVE_CUBE,
    materialID=TRANSPARENT_BLACK,
    pose=Pose(
        position=Vector3((-box_length/2)+(box_width/2), box_height/4,(-box_length/2)+(box_width/2)),
        rotation=Quaternion(0, 1, 0, 1),
        scale=Vector3(box_length, box_width/2, box_height)  # Scale appropriately for rectangular box
    ),
    isManipulationOn=False,
)

box4 = PrimitiveMessage(
    id="box4",
    source=PRIMITIVE_CUBE,
    materialID=TRANSPARENT_BLACK,
    pose=Pose(
        position=Vector3(0, box_height/4,(-box_length)+box_width),
        rotation=Quaternion(1, 0, 0, 1),
        scale=Vector3(box_length, box_width, box_height/2)  # Scale appropriately for rectangular box
    ),
    isManipulationOn=False,
)
# Post the box object initially
createResponse = cwruxrClient.PostObjectBulk([box1,box2,box3,box4])

# Particle properties and simulation parameters
num_particles = 50
radius_particle = 0.05
masses = cp.ones(num_particles)
dt = 0.01
dissipation = 0.925

positions = cp.array([[random.uniform((-box_length/2)+.1, (-box_length/2)+box_width-.1), 
                       random.uniform(0.1, (box_height/2)-.1),
                       random.uniform((-box_width/2)+.1, (box_width/2)-.1)] for _ in range(num_particles)])
velocities = cp.array([[random.uniform(1.0,1.0), random.uniform(0.3, .50), random.uniform(1.0, 1.0)] for _ in range(num_particles)])

# Handle particle-particle collisions
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
                velocities[p1[i]] += impulse * masses[p2[i]] * normal * dissipation
                velocities[p2[i]] -= impulse * masses[p1[i]] * normal * dissipation

first_corner_bounds = {
    'x': ((-box_length/2), (-box_length/2)+box_width),
    'y': (0.0, (box_height/2)),
    'z': (-box_width/2, box_width/2)}

second_corner_bounds = {
    'x': ((box_length/2)-box_width, (box_length/2)),
    'y': (0.0, (box_height/2)),
    'z': (-box_width/2, box_width/2)}

third_corner_bounds = {
    'x': ((box_length/2)-box_width, (box_length/2)),
    'y': (0.0, (box_height/2)),
    'z': (-box_length+(box_width/2),-box_length+(3*box_width/2))}

fourth_corner_bounds = {
    'x': ((-box_length/2), (-box_length/2)+box_width),
    'y': (0.0, (box_height/2)),
    'z': (-box_length+(box_width/2),-box_length+(3*box_width/2))}

# Define a force vector pushing towards positive x (from corner 1 to corner 2)
force_vector_1 = cp.array([0.05, 0.0, -.03])  # Adjust the magnitude as needed
force_vector_2 = cp.array([-0.03, 0.0, -0.05])  
force_vector_3 = cp.array([-0.05, 0.0, 0.03])
force_vector_4 = cp.array([0.03, 0.0, 0.05])

# Update the update_positions_and_check_walls function to apply force in the first corner
def update_positions_and_check_walls(positions, velocities, radius_particle, outer_box, inner_box):
    positions += velocities * dt * dissipation
    
    # Outer and inner box boundary checks remain the same
    
    # Apply force in the first corner
    in_first_corner = (
        (positions[:, 0] >= first_corner_bounds['x'][0]) & (positions[:, 0] <= first_corner_bounds['x'][1]) &
        (positions[:, 1] >= first_corner_bounds['y'][0]) & (positions[:, 1] <= first_corner_bounds['y'][1]) &
        (positions[:, 2] >= first_corner_bounds['z'][0]) & (positions[:, 2] <= first_corner_bounds['z'][1])
    )

    in_second_corner = (
        (positions[:, 0] >= second_corner_bounds['x'][0]) & (positions[:, 0] <= second_corner_bounds['x'][1]) &
        (positions[:, 1] >= second_corner_bounds['y'][0]) & (positions[:, 1] <= second_corner_bounds['y'][1]) &
        (positions[:, 2] >= second_corner_bounds['z'][0]) & (positions[:, 2] <= second_corner_bounds['z'][1])
    )
    
    in_third_corner = (
        (positions[:, 0] >= third_corner_bounds['x'][0]) & (positions[:, 0] <= third_corner_bounds['x'][1]) &
        (positions[:, 1] >= third_corner_bounds['y'][0]) & (positions[:, 1] <= third_corner_bounds['y'][1]) &
        (positions[:, 2] >= third_corner_bounds['z'][0]) & (positions[:, 2] <= third_corner_bounds['z'][1])
    )

    in_fourth_corner = (
        (positions[:, 0] >= fourth_corner_bounds['x'][0]) & (positions[:, 0] <= fourth_corner_bounds['x'][1]) &
        (positions[:, 1] >= fourth_corner_bounds['y'][0]) & (positions[:, 1] <= fourth_corner_bounds['y'][1]) &
        (positions[:, 2] >= fourth_corner_bounds['z'][0]) & (positions[:, 2] <= fourth_corner_bounds['z'][1])
    )
    # Add force to particles in the first corner
    velocities[in_first_corner] += force_vector_1
    velocities[in_second_corner] += force_vector_2
    velocities[in_third_corner] += force_vector_3
    velocities[in_fourth_corner] += force_vector_4

    # Outer box boundaries
    x_min_outer, x_max_outer = outer_box['x']
    y_min_outer, y_max_outer = outer_box['y']
    z_min_outer, z_max_outer = outer_box['z']
    
    # Inner box boundaries
    x_min_inner, x_max_inner = inner_box['a']
    z_min_inner, z_max_inner = inner_box['c']
    
    # Outer box collision handling
    out_of_bounds_x_outer = (positions[:, 0] >= x_max_outer - radius_particle) | (positions[:, 0] <= x_min_outer + radius_particle)
    out_of_bounds_y_outer = (positions[:, 1] >= y_max_outer - radius_particle) | (positions[:, 1] <= y_min_outer + radius_particle)
    out_of_bounds_z_outer = (positions[:, 2] >= z_max_outer - radius_particle) | (positions[:, 2] <= z_min_outer + radius_particle)
    
    # Reverse velocities for outer box boundaries
    velocities[out_of_bounds_x_outer, 0] *= -dissipation
    velocities[out_of_bounds_y_outer, 1] *= -dissipation
    velocities[out_of_bounds_z_outer, 2] *= -dissipation
    
    # Inner box collision handling only if particles are within the inner region
    within_inner_region = (positions[:, 0] >= x_min_inner) & (positions[:, 0] <= x_max_inner) & \
                          (positions[:, 2] >= z_min_inner) & (positions[:, 2] <= z_max_inner) 
    # Side wall hits in the inner box
    side_wall_hit_inner = (positions[:, 0] >= x_max_inner - radius_particle) | (positions[:, 0] <= x_min_inner + radius_particle)
    velocities[side_wall_hit_inner & within_inner_region, 0] *= -dissipation
    
    # Front/back wall hits in the inner box
    front_back_wall_hit_inner = (positions[:, 2] >= z_max_inner - radius_particle) | (positions[:, 2] <= z_min_inner + radius_particle)
    velocities[front_back_wall_hit_inner & within_inner_region, 2] *= -dissipation

# Input thread for interactive commands
stop_simulation = False
path_posted = True

def check_pos(positions, velocities, outer_box, inner_box):
    # Outer box boundaries
    x_min_outer, x_max_outer = outer_box['x']
    y_min_outer, y_max_outer = outer_box['y']
    z_min_outer, z_max_outer = outer_box['z']
    
    # Inner box boundaries
    x_min_inner, x_max_inner = inner_box['a']
    z_min_inner, z_max_inner = inner_box['c']
    
    # Create masks for particles outside the outer box or inside the inner box
    outside_mask = (positions[:, 0] > x_max_outer - 0.024) | (positions[:, 0] < x_min_outer + 0.024) | \
                   (positions[:, 1] > y_max_outer) | (positions[:, 1] < y_min_outer ) | \
                   (positions[:, 2] > z_max_outer - 0.03) | (positions[:, 2] < z_min_outer + 0.024)
    
    inside_inner_box_mask = (positions[:, 0] > x_min_inner) & (positions[:, 0] < x_max_inner) & \
                            (positions[:, 2] > z_min_inner + .01) & (positions[:, 2] < z_max_inner - .01)    
    for i in range(len(positions)):
        # If the particle is outside the outer box or inside the inner box
        if outside_mask[i] or inside_inner_box_mask[i]:
            # Delete the particle from the CWRUXR environment
            delete_response = cwruxrClient.DeleteObject(f"particle{i + 1}")
            
            # Respawn at a random position within the firzst corner
            new_position = cp.array([random.uniform((-box_length/2)+.1, (-box_length/2)+box_width-.1), 
                       random.uniform(0.1, (box_height/2)-.1),
                       random.uniform((-box_width/2)+.1, (box_width/2)-.1)])
            positions[i] = new_position

            # Reset velocity for this particle
            velocities[i] = cp.array([random.uniform(1.0, 1.0), random.uniform(0.3, .50), random.uniform(1.0, 1.0)])

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

def input_thread():
    global stop_simulation, path_posted
    while True:
        user_input = input("Enter 'p' to post the path, 'd' to delete it:")
        if user_input == 'p':
            if not path_posted:
                print("Posting the path.")
                createResponse = cwruxrClient.PostObjectBulk([box1,box2,box3,box4])
                path_posted = True
        elif user_input == 'd':
            if path_posted:
                print("Deleting the path.")
                deleteResponse = cwruxrClient.DeleteObjectBulk(["box1","box2","box3","box4"])
                path_posted = False

# Start input thread
thread = threading.Thread(target=input_thread)
thread.start()

# Real-time simulation loop
outer_box = {'x': (-box_length/2, box_length/2), 'y': (0, (box_height/2)-radius_particle), 'z': (-box_length+ (box_width/2), (box_width/2))}
inner_box = {'a': ((-box_length/2)+box_width-radius_particle, (box_length/2)-box_width+radius_particle), 'c': ((-box_length+ (3*box_width/2))-radius_particle, (-box_width/2)+radius_particle)}


try:
    while not stop_simulation:
        start_time = time.time()

        update_positions_and_check_walls(positions, velocities, radius_particle, outer_box, inner_box)
        handle_collisions(positions, velocities, masses, radius_particle)
        check_pos(positions, velocities, outer_box, inner_box)

        #check_grabbed_particles(positions, velocities)
        end_time = time.time()
        calc_duration = end_time - start_time
        #print(f"Calculation time for this iteration: {calc_duration:.6f} seconds")

        # Send particle updates to CWRUXR API
        particles = [
            PrimitiveMessage(
                id=f"particle{j + 1}",
                source=PRIMITIVE_SPHERE,
                materialID=LIT_BLUE,
                pose=Pose(
                    position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                    rotation=Quaternion(0, 0, -1, 0),
                    scale=Vector3(radius_particle*2, radius_particle*2, radius_particle*2)
                ),
                isManipulationOn=True
            ) for j, pos in enumerate(cp.asnumpy(positions))
        ]

        cwruxrClient.PostObjectBulk(particles)
        
        time.sleep(dt)

except KeyboardInterrupt:
    print("Simulation stopped by user.")

# Cleanup after stopping
thread.join()
cwruxrClient.DeleteAllObjects()