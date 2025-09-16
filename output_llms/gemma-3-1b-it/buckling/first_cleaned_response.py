import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.simulation import Simulation


try:
    simulation = Simulation()
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()



beam = Simulation.Beam(
    length=1.0,  
    cross_section='I'  
)


support1 = Simulation.Support(
    position=[0, 0],  
    type='fixed'
)
support2 = Simulation.Support(
    position=[0, 0],  
    type='fixed'
)


gravity = 9.81  


simulation.set_default_parameters(
    gravity=gravity,
    beam_length=1.0,
    support_positions=[0, 0],
    support_types=['fixed'],
    beam_type='rectangular'
)


simulation.create_data(
    beam,
    support1,
    support2,
    gravity,
    0.0,  
    1.0,  
    1000, 
    1,  
    10, 
    1, 
    0.01, 
    100, 
    1000 
)


def apply_force(beam, force):
    
    
    print(f"Applying force: {force} N to beam")
    return force


try:
    plt.figure(figsize=(10, 6))
    plt.plot(simulation.time, simulation.beam.position, label='Beam Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Beam Buckling Simulation')
    plt.legend()
    plt.grid(True)
    plt.show()

except Exception as e:
    print(f"Error creating visualization: {e}")
    print("Ensure Irrlicht is installed and configured correctly.")


try:
    simulation.set_solver(Simulation.Solver.Euler)  
    simulation.set_timestepper(100) 
except Exception as e:
    print(f"Error setting solver/timestepper: {e}")
    exit()


simulation.run()
print("Simulation completed.")