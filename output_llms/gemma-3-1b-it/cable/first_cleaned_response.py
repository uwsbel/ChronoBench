import pychrono
import pychrono.core as pc
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


beam_length = 1.0  
cable_radius = 0.05  
gravity = 9.81  
cable_strength = 1000.0  
num_elements = 10  
initial_position = [0.0, 0.0]  
initial_velocity = 0.0  

env = pc.Environment()


beam = env.create_beam(beam_length, cable_radius, cable_strength)


for i in range(num_elements):
    beam[i].position = initial_position[i]


for i in range(num_elements):
    beam[i].velocity = initial_velocity[i]


env.set_time_step(0.001)  
env.set_simulation_time(0, 100)  
env.set_physics_parameters(
    physics_type="static",
    gravity=gravity,
    cable_strength=cable_strength,
    beam_length=beam_length
)


def update(dt):
    
    for i in range(num_elements):
        
        force = np.array([0.0, 0.0, 0.0])
        moment = np.array([0.0, 0.0, 0.0])

        
        for j in range(num_elements):
            if i != j:
                force[i] += beam[i].velocity[j] * cable_strength * (beam[i].position[j] - beam[i].position[j])
                moment[i] += beam[i].velocity[j] * cable_strength * (beam[i].position[j] - beam[i].position[j])

        
        beam[i].velocity[i] += force[i] * dt
        beam[i].moment[i] += force[i] * dt

        
        beam[i].position[i] += beam[i].velocity[i] * dt

    return beam


fig, ax = plt.subplots()
ax.set_xlim(0, beam_length)
ax.set_ylim(0, beam_length)
ax.set_aspect('equal')
ax.set_title("Beam Simulation")
ax.set_xlabel("X")
ax.set_ylabel("Y")


beam_obj = pc.Beam(beam, cable_strength)


def animate(i):
    
    beam_obj.update(dt)
    return beam_obj


ani = animation.FuncAnimation(fig, animate, frames=100, blit=True, interval=dt*1000/1000)  

plt.show()