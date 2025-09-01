import pychrono
import pychrono.core as pc
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


try:
    
    chrono = pc.Chrono()

    
    system_params = {
        "name": "Beam",
        "type": "ANCF",
        "num_elements": 10,  
        "cable_length": 1.0, 
        "cable_radius": 0.1, 
        "gravity_magnitude": 9.81, 
        "gravity_direction": (0, 0, -1) 
    }

    
    
    elements = [
        pc.Element("cable",  
                 cable_length=system_params["cable_length"],
                 cable_radius=system_params["cable_radius"],
                 gravity_direction=system_params["gravity_direction"],
                 mass=1000.0, 
                 num_segments=5) 

    ]


    
    
    simulation_time = 10.0  
    chrono.set_simulation_time(simulation_time)

    
    fig, ax = plt.subplots()
    ax.set_xlim(-1.5, 1.5)  
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.set_title("Beam Simulation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)

    def animate(i):
        
        element = elements[i]
        x = element.x
        y = element.y
        
        
        element.x += element.gravity_direction[0] * simulation_time
        element.y += element.gravity_direction[1] * simulation_time
        
        
        ax.clear() 
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect('equal')
        ax.set_title("Beam Simulation")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True)
        
        
        element.x = x
        element.y = y
        
        return ax

    ani = animation.FuncAnimation(fig, animate, frames=len(elements), blit=True, repeat=False)
    plt.show()

except Exception as e:
    print(f"Error initializing Chrono: {e}")
    print("Please ensure PyChrono is installed correctly and that your system has the necessary dependencies.")
    print("Consider installing 'pychrono-dependencies' if you encounter issues.")