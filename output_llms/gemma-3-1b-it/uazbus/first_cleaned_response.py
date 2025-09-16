import pychrono
import pychrono.core as chrono
import pychrono.visual as visual
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


initial_position = [0, 0, 0]
initial_velocity = [10, 0, 0]
friction = 0.8
restitution = 0.7
terrain_height = 5
terrain_scale = 10
time_step = 0.1
simulation_duration = 60  


vehicle = chrono.Vehicle(
    initial_position,
    initial_velocity,
    friction=friction,
    restitution=restitution,
    mass=100, 
    vehicle_type="UAZBUS"
)


terrain = chrono.Terrain(
    height=terrain_height,
    scale=terrain_scale,
    friction=0.5,  
    restitution=0.6, 
    
)


driver = chrono.Driver(
    vehicle,
    initial_position,
    initial_velocity,
    
)



fig, ax = visual.figure(figsize=(8, 6))
camera = visual.Camera(
    center=(0, 0),
    farce=0.5,
    zoom=1.0,
    alpha=0.5,
    width=800,
    height=600
)

def update(frame):
    
    vehicle.position = np.array(vehicle.position)
    vehicle.velocity = np.array(vehicle.velocity)

    
    terrain.update()

    
    driver.update()

    
    camera.update()

    return fig, ax, camera


ani = visual.FuncAnimation(fig, update, frames=simulation_duration, interval=10, blit=True)


ani.save("simulation.mp4", fps=30, extra_args=['-movaprefetch'])


plt.show()