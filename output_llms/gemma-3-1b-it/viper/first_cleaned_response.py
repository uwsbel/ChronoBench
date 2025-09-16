import pychrono
import pychrono.core as pc
import irlicht as il
import numpy as np


time_step = 0.1
simulation_duration = 10  
gravity = 9.81  
ground_body_mass = 1000  
ground_body_radius = 1.0 
vibration_frequency = 1.0  
vibration_amplitude = 0.5 
vibration_speed = 0.2 


chrono = pc.Chrono()


terrain_heightmap = np.zeros((100, 100))
terrain_heightmap[50:95, 50:95] = 1.0  


rover = pc.Rover()
rover.name = "Viper"
rover.position = np.array([0, 0, 0])
rover.gravity = gravity
rover.ground_body_mass = ground_body_mass
rover.ground_body_radius = ground_body_radius
rover.vibration_frequency = vibration_frequency
rover.vibration_amplitude = vibration_amplitude
rover.vibration_speed = vibration_speed


camera = il.Camera(
    width=800,
    height=600,
    camera_type='perspective',
    camera_position = (0, 0, 10),
    camera_tilt = 30,
    camera_zoom = 1.0,
    camera_rotation = np.array([0, 0, 0])
)


def update_simulation():
    global rover

    
    rover.position = rover.position + rover.vibration_speed * time_step

    
    terrain_heightmap[rover.position[0], rover.position[1]] = 0.0

    
    camera.render(terrain_heightmap, camera_position, camera_tilt, camera_zoom, camera_rotation)

    
    chrono.update()


update_simulation()