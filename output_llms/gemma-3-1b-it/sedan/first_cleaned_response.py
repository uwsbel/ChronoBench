Okay, let's craft a PyChrono simulation script to model a BMW E90 Sedan driving on a rigid terrain. This will be a detailed breakdown, incorporating the core elements you've outlined.  I'll prioritize clarity and modularity, making it easy to expand and customize.

```python
import pychrono
import pychrono.core as chrono
import pychrono.visuals as visuals
import pychrono.physics as physics
import pychrono.ui as ui
import numpy as np
import matplotlib.pyplot as plt  





terrain_height = 0.05  
terrain_width = 10.0  
terrain_length = 20.0 
terrain_slope = 0.01 


vehicle_id = 1  
vehicle_mass = 1500.0  
vehicle_position = (0.0, 0.0, 0.0)  
vehicle_velocity = (0.0, 0.0, 0.0) 
vehicle_acceleration = (0.0, 0.0, 0.0) 
vehicle_brake = (0.0, 0.0, 0.0) 




tire_model = "TMEASY"
tire_radius = 0.3 
tire_height = 0.05 


collision_threshold = 0.1 
collision_radius = 0.1 




simulation_time = 60.0  
simulation_steps = 100
simulation_rate = 1.0 




visuals.create_visual(
    "BMW_E90_Simulation",