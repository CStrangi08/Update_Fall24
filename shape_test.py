import cupy as cp
import time
from cwruxr_sdk.client import Client
from cwruxr_sdk.common import Pose, Vector3, Quaternion
from cwruxr_sdk.object_message import PrimitiveMessage, PRIMITIVE_SPHERE, PRIMITIVE_CUBE
from cwruxr_sdk.built_in_assets import LIT_BLUE, TRANSPARENT_BLACK,TRANSPARENT_BLUE
import threading
import random

box_length = 6.0
box_height = 1.5 #Note, height and width must be the same
box_width = 1.5

ENDPOINT = "https://cwruxr.azurewebsites.net/api/v3/"
ANCHOR_CODE = "SFHZ"
API_KEY = "FCF0D275B2AA0769DC1D7F7941824E"
cwruxrClient = Client(ENDPOINT, ANCHOR_CODE, API_KEY)

# Box object definition 
box1 = PrimitiveMessage(
    id="box1",
    source=PRIMITIVE_CUBE,
    materialID=TRANSPARENT_BLACK,
    pose=Pose(
        position=Vector3(0, box_height/4,0),
        rotation=Quaternion(1, 0, 0, 1),
        scale=Vector3(box_length, box_width, box_height/2)  # Scale appropriately for rectangular box
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

radius_particle = 0.05

# Real-time simulation loop
outer_box = {'x': (-box_length/2, box_length/2), 'y': (0, (box_height/2)-radius_particle), 'z': (-box_length+ (box_width/2), (box_width/2))}
inner_box = {'a': ((-box_length/2)+box_width-radius_particle, (box_length/2)-box_width+radius_particle), 'c': ((-box_length+ (3*box_width/2))-radius_particle, (-box_width/2)+radius_particle)}
