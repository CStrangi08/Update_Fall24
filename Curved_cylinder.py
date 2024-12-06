from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_TORUS, PRIMITIVE_SPHERE
from cwruxr_sdk.built_in_assets import LIT_BLUE, TRANSPARENT_BLACK
import time
import numpy as np
import random


import numpy as np

import math

# Parameters
radius = 1.0           # Radius of the circular path
total_time = 5.0       # Total time for simulation in seconds
dt = 0.01              # Time increment in seconds
num_steps = int(total_time / dt)  # Total number of steps

# List to hold positions
positions = []

# Calculate positions for every dt
for i in range(num_steps + 1):  # +1 to include the last position at total_time
    theta = 2 * math.pi * (i * dt) / total_time  # Calculate angle for circular motion
    x = radius * math.cos(theta)                   # x position
    y = radius * math.sin(theta)                   # y position
    positions.append((x, y))                        # Append the position as a tuple

# Example of how to use the positions in a for loop



ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "JSTK"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"

cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

for i in range(len(positions)):
    cube = PrimitiveMessage(
    id = "cube1",
    source = PRIMITIVE_SPHERE,
    materialID = LIT_BLUE,
    pose=Pose(
            position=Vector3(float(positions[i][0]), float(positions[i][1]),1),
            rotation=Quaternion(0, 0, -1, 0),
            scale=Vector3(.1, .1, .1)
        ),
    isManipulationOn = True,
)
    
createResponse = cwruxrClient.PostObject(cube)

input("Press Enter to Delete")

deleteResponse = cwruxrClient.DeleteObject(cube.id)
