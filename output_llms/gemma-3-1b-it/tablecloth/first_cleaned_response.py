import pychrono
import pychrono.core as pc
import pychrono.physics as pcp
import matplotlib.pyplot as plt
import numpy as np


grid_size = 100  
element_size = 0.1  
material_type = 'isotropic_kirkegaard'
force_magnitude = 1000  
time_step = 0.01  
simulation_duration = 10  


chrono.init()


mesh = pc.Mesh(grid_size, element_size)


mesh.add_nodes()
mesh.add_elements()


chrono.set_material(mesh, material_type)


chrono.set_initial_conditions(mesh)


chrono.set_simulation_params(
    time_step,
    simulation_duration,
    force_magnitude,
    grid_size,
    element_size
)


plt.figure(figsize=(8, 6))
plt.imshow(mesh, cmap='gray')
plt.title('Cloth Tabby Folding Simulation')
plt.colorbar(label='Force')
plt.show()


chrono.run()

print("Simulation completed.")