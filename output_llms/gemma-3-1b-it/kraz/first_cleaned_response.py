import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


try:
    chrono = pc.chrono.Chrono()
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()



vehicle = pc.Vehicle(
    x=0.0,
    y=0.0,
    vx=0.0,
    vy=0.0,
    mass=10.0,  
    friction=0.8,
    restitution=0.7,
    initial_position=(0.0, 0.0) 
)


terrain = pc.Terrain(
    x_min=-10.0,
    x_max=10.0,
    y_min=-10.0,
    y_max=10.0,
    friction=0.9,
    restitution=0.8,
    initial_position=(0.0, 0.0)
)


driver = pc.Driver(
    vehicle=vehicle,
    initial_position=(0.0, 0.0),
    control_method="manual", 
    max_speed=5.0, 
    acceleration=0.1, 
    deceleration=0.1, 
    max_deceleration=1.0, 
    simulation_time=10.0 
)


chrono.set_default_parameters(
    time_step=0.01, 
    initial_velocity=0.0, 
    gravity=9.81,
    friction=0.8,
    restitution=0.7
)


fig, ax = v.figure(width=800, height=600)
scene = v.Scene()


camera = v.Camera(
    x=0.0,
    y=0.0,
    z=0.0,
    width=800,
    height=600,
    fov=60.0,
    aspect=2.0,
    near=-10.0,
    far=10.0,
    color=(1.0, 1.0, 1.0),  
    animation=True
)


def animate(i):
    camera.set_position(camera.position)
    camera.set_rotation(camera.rotation)
    
    
    camera.update()
    
    return scene.animate(camera,  
                     lambda i=i:  
                     animate(i))


ani = v.FuncAnimation(fig, animate, frames=chrono.get_frame_count(),
                      interval=10,
                      repeat=False)


v.show(fig, scene, title="Kraz Simulation")


chrono.save_animation(ani, "kraz_simulation.mp4")
print("Simulation complete. Animation saved to kraz_simulation.mp4")