import pychrono
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


simulation_time = 10  
time_step = 0.1  
initial_position = [0, 0]  
initial_velocity = [10, 0]  
friction = 0.8  
restitution = 0.7  
terrain_friction = 0.5  
terrain_restitution = 0.8  
driving_speed = 5  
terrain_height = 10 


chrono = pychrono.Chronos()


terrain = chrono.create_terrain(
    width=100,
    height=10,
    friction=terrain_friction,
    restitution=terrain_restitution
)


uzazbus = chrono.create_object(
    name="uzazbus",
    position=initial_position,
    velocity=initial_velocity,
    mass=100,
    driving_speed=driving_speed,
    terrain=terrain
)


driver_system = chrono.create_driver(
    name="driver",
    position=initial_position,
    velocity=initial_velocity,
    acceleration=0.1,
    max_speed=driving_speed
)


fig, ax = plt.subplots()
ax.set_xlim(0, 100)
ax.set_ylim(0, 10)
ax.set_aspect('equal')
ax.set_title('UAZBUS Simulation')
ax.set_xlabel('X')
ax.set_ylabel('Y')


def init():
    
    scene = ax.copy()
    scene.set_xlim(0, 100)
    scene.set_ylim(0, 10)
    scene.set_aspect('equal')
    scene.set_title('UAZBUS Simulation')
    return scene

def animate(i):
    
    uzazbus.position = initial_position + uzazbus.velocity * time_step

    
    terrain.update()

    
    scene.clear()
    ax.clear()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    ax.set_title('UAZBUS Simulation')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    return scene,


ani = FuncAnimation(fig, animate, frames=100, init_func=init, blit=True, interval=10)


ani.save("uzazbus_animation.mp4", writer='ffmpeg', fps=30)


plt.show()