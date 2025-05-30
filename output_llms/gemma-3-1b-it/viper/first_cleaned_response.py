import pychrono
import pychrono.core as chrono
import pychrono.visuals as visual
import pyrr
import time
import random


gravity = 9.81  
ground_body_mass = 1000  
ground_body_radius = 1.0  
rover_mass = 150  
rover_speed = 0.5  
simulation_duration = 60  
time_step = 0.1  
simulation_rate = 10  


chrono.init()



terrain_width = 10
terrain_height = 10
terrain_x = 0
terrain_y = 0
terrain_z = 0


terrain = chrono.Terrain(
    x=terrain_x,
    y=terrain_y,
    z=terrain_z,
    width=terrain_width,
    height=terrain_height,
    density=0.5,  
    
    
    
)


rover = chrono.Rover(
    mass=rover_mass,
    position= pyrr.Vector(terrain_x, terrain_y, terrain_z),
    speed=rover_speed,
    orientation=chrono.Orientation(angle=0,  
                             axis= pyrr.Vector(0, 0, 0))
)


camera_resolution = 800
camera_fov = 60  
camera_aspect_ratio = 1.0  
camera_z_offset = 0.0  


visual.init(camera_resolution=camera_resolution,
            camera_aspect_ratio=camera_aspect_ratio,
            camera_z_offset=camera_z_offset)


for i in range(simulation_rate):
    
    rover.move(i * time_step)

    
    ground_body_position = rover.position.copy()
    ground_body_position.x += random.uniform(-0.1, 0.1)  
    ground_body_position.y += random.uniform(-0.1, 0.1)
    ground_body_position.z += random.uniform(-0.1, 0.1)

    
    
    
    force_magnitude = 0.1
    force_direction = ground_body_position.z - rover.position.z
    
    
    rover.apply_force(force_magnitude * force_direction * time_step)


    
    visual.render(terrain, rover, camera)

    
    time.sleep(time_step)

print("Simulation complete.")