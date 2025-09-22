import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System
from pychrono.core.physics import Physics
from pychrono.core.simulation import Simulation


try:
    system = System(
        name="BeamSimulation",
        physics=Physics(
            
            physics_type="beam",
            
            mesh="beam_mesh.npy",  
            
            nodes=100,
            
            elements=50,
            
            element_type="linear",
            
            element_size=0.1,
            
            boundary_conditions="fixed",
            
            boundary_conditions_type="fixed",
            
            material_properties={"Young's Modulus": 2.0e9}
        ),
        
        parameters={"time_step": 0.01, "max_time": 10.0}
    )
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()




beam_elements = [
    {
        "name": "Element1",
        "type": "linear",
        "size": 0.1,
        "element_type": "linear",
        "element_size": 0.1,
        "boundary_conditions": "fixed",
        "material_properties": {"Young's Modulus": 2.0e9}
    },
    {
        "name": "Element2",
        "type": "linear",
        "size": 0.1,
        "element_type": "linear",
        "element_size": 0.1,
        "boundary_conditions": "fixed",
        "material_properties": {"Young's Modulus": 2.0e9}
    },
    
]



system.time_step = 0.01
system.max_time = 10.0
system.boundary_conditions = "fixed"
system.boundary_conditions_type = "fixed"


try:
    simulation = System(
        name="BeamSimulation",
        physics=Physics(
            
            physics_type="beam",
            
            mesh="beam_mesh.npy",  
            
            nodes=100,
            
            elements=50,
            
            element_type="linear",
            
            element_size=0.1,
            
            boundary_conditions="fixed",
            
            boundary_conditions_type="fixed",
            
            material_properties={"Young's Modulus": 2.0e9}
        ),
        
        parameters={"time_step": 0.01, "max_time": 10.0}
    )
except Exception as e:
    print(f"Error creating simulation: {e}")
    exit()



try:
    simulation.run(
        
        time_step=system.time_step,
        max_time=system.max_time
    )
except Exception as e:
    print(f"Error during simulation: {e}")
    exit()


try:
    plt.figure(figsize=(8, 6))
    plt.imshow(system.physics.nodes, cmap='viridis', interpolation='nearest')
    plt.title("Beam Simulation")
    plt.colorbar(label="Node Value")
    plt.show()
except Exception as e:
    print(f"Error displaying visualization: {e}")