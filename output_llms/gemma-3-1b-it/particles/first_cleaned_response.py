import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.simulation import Simulation


try:
    simulation = Simulation()
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()



particles = []
num_particles = 100

for i in range(num_particles):
    position = np.random.rand(3) * 10  
    velocity = np.random.rand(3) * 5  
    orientation = np.random.rand(3) * 2 * np.pi  
    
    particles.append({
        'position': position,
        'velocity': velocity,
        'orientation': orientation
    })


gravity_strength = 10  
gravity_radius = 1.0  

for particle in particles:
    
    force = np.array([0.0, 0.0, gravity_strength])
    
    
    particle['force'] = force
    
    
    particle['position'] = particle['position'] + particle['force'] * particle['velocity']
    
    
    particle['orientation'] = particle['orientation'] + particle['force'] * particle['velocity']
    
    
    particle['acceleration'] = -gravity_strength * particle['position']
    

fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect('equal')
ax.set_title('Particle Attraction Simulation')

for particle in particles:
    particle['color'] = 'blue'
    ax.plot(particle['position'], particle['orientation'], 'r-', lw=2)  
    

for i in range(len(particles)):
    ax.scatter(particles[i]['position'], particles[i]['orientation'], color='gray', s=50)

plt.show()


simulation.run(num_particles)

print("Simulation complete.")