import pychrono
import time
import random
import math


SIMULATION_TIME = 100  
INITIAL_POSITION = (0, 0, 0)  
INITIAL_SPEED = 0.1  
GRAVITY = 9.81
ROCKET_MASS = 1.0  
MOTOR_CONTROL_SPEED = 0.05 
ROCKET_ENGINED_INITIAL_FORCE = 0.0  
ROCKET_ENGINED_MAX_FORCE = 1.0 
ROCKET_ENGINED_INITIAL_POSITION = (0,0,0) 
ROCKET_ENGINED_SPEED = 0.0 


terrain_height = 2.0
terrain_width = 10.0
terrain_depth = 1.0
terrain_resolution = 100  


rover = pychrono.Objects.Rover()
terrain = pychrono.Objects.Terrain(
    width=terrain_width,
    depth=terrain_depth,
    resolution=terrain_resolution,
    height=terrain_height,
    mass=ROCKET_MASS,
    friction=0.1,
    color="grey"
)


camera_x = 0.0
camera_y = 0.0
camera_z = 0.0
camera_fov = 60.0
camera_aspect_ratio = 1.0
camera_zoom = 1.0
camera_color = "white"


visual_width = 800
visual_height = 600
visual_resolution = 300
visual_color = "white"
visual_camera_x = 0.0
visual_camera_y = 0.0
visual_camera_z = 0.0
visual_camera_fov = 60.0
visual_camera_aspect_ratio = 1.0
visual_zoom = 1.0
visual_camera_color = "white"


state = pychrono.State(
    position=INITIAL_POSITION,
    velocity=INITIAL_SPEED,
    orientation=pychrono.Objects.Orientation.NONE,
    force=ROCKET_ENGINED_INITIAL_FORCE,
    engine_speed=ROCKET_ENGINED_INITIAL_SPEED,
    terrain=terrain,
    camera=camera_x,
    camera_y=camera_y,
    camera_z=camera_z,
    visual=visual_width,
    visual_height,
    visual_resolution,
    visual_color,
    time_step=1/SIMULATION_TIME
)



for t in range(SIMULATION_TIME):
    
    state.velocity += state.force * time_step
    state.velocity = max(0.0, state.velocity) 

    
    state.velocity += terrain.gravity * time_step
    state.velocity = max(0.0, state.velocity)

    
    state.force = state.velocity * ROCKET_ENGINED_INITIAL_FORCE
    state.force = max(0.0, state.force)

    
    state.engine_speed = state.velocity * ROCKET_ENGINED_INITIAL_SPEED

    
    if state.position[0] < 0 or state.position[0] > terrain_width or state.position[1] < 0 or state.position[1] > terrain_height:
        print("Collision detected!")
        state.force = 0.0
        state.velocity = 0.0
        break

    
    state.visual.color = visual_color
    state.visual.camera_x = visual_camera_x
    state.visual.camera_y = visual_camera_y
    state.visual.camera_z = visual_camera_z
    state.visual.fov = visual_camera_fov
    state.visual.aspect_ratio = visual_camera_aspect_ratio
    state.visual.zoom = visual_zoom
    state.visual.camera_color = visual_color

    
    print(f"Time: {t}")

    
    if state.position[0] < 0:
        print("Rover crashed!")
        break
    if state.position[1] > terrain_height:
        print("Rover went off the terrain!")
        break

    
    state = state.update()


print("Simulation finished.")