import pychrono
import pychrono.core as pc
import pychrono.visual as pv
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual import MeshVisualizer
from pychrono.core import Simulation
from pychrono.visual import MeshVisualizer
import math


simulation_name = "HMMWV_Simulation"
simulation_time = 50  
frame_rate = 50
terrain_resolution = 10  
vehicle_position = [0, 0, 0]
vehicle_orientation = [0, 0, 0]
contact_method = pc.ContactMethod.STABLE  
tmeasy_tire_model = pc.TMEasyTireModel.MODEL_NORMAL
vehicle_mass = 1000  
vehicle_speed = 1.0  
vehicle_acceleration = 0.1  
vehicle_weight = vehicle_mass



highway_col = pc.Mesh(
    "Highway_col.obj",
    
    vertices=[
        [0, 0, 0],
        [10, 0, 0],
        [10, 10, 0],
        [0, 10, 0],
        [0, 0, 10],
        [10, 0, 10],
        [10, 10, 10],
        [0, 10, 10],
    ],
    
    faces=[
        [0, 1, 2], [0, 2, 3],
        [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4],
        [2, 3, 7], [2, 7, 6],
        [3, 7, 4], [3, 4, 1],
        [0, 4, 2], [0, 2, 3]
    ]
)


terrain_vis = pc.MeshVisualizer(
    "Highway_vis.obj",
    
    vertices=[
        [0, 0, 0],
        [10, 0, 0],
        [10, 10, 0],
        [0, 10, 0],
        [0, 0, 10],
        [10, 0, 10],
        [10, 10, 10],
        [0, 10, 10],
    ],
    
    faces=[
        [0, 1, 2], [0, 2, 3],
        [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4],
        [2, 3, 7], [2, 7, 6],
        [3, 7, 4], [3, 4, 1],
        [0, 4, 2], [0, 2, 3]
    ]
)


simulation = Simulation(
    name=simulation_name,
    time=simulation_time,
    frame_rate=frame_rate,
    terrain_resolution=terrain_resolution,
    vehicle_position=vehicle_position,
    vehicle_orientation=vehicle_orientation,
    contact_method=contact_method,
    tmeasy_tire_model=tmeasy_tire_model,
    vehicle_mass=vehicle_mass,
    vehicle_speed=vehicle_speed,
    vehicle_weight=vehicle_weight,
    simulation_time=simulation_time,
    terrain_vis=terrain_vis,
    simulation_name=simulation_name
)


for i in range(simulation_time):
    
    vehicle_acceleration = vehicle_acceleration * (i / simulation_time)
    vehicle_velocity += vehicle_acceleration * (i / simulation_time)
    vehicle_position += vehicle_velocity * (i / simulation_time)

    
    if vehicle_position[0] < 0:
        vehicle_position[0] = -vehicle_mass
    elif vehicle_position[0] > 10:
        vehicle_position[0] = 10 - vehicle_mass
    elif vehicle_position[1] < 0:
        vehicle_position[1] = -vehicle_mass
    elif vehicle_position[1] > 10:
        vehicle_position[1] = 10 - vehicle_mass

    
    if vehicle_position[0] < 0:
        contact_method(vehicle_position, terrain_vis)
    elif vehicle_position[0] > 10:
        contact_method(vehicle_position, terrain_vis)

    
    pv.render(simulation.get_scene(), resolution=terrain_resolution)

    
    print(f"Frame {i}")


simulation.shutdown()