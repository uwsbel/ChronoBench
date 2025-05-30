import pychrono
import pychrono.core as pc
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- 1. Initialization & Core Components ---
try:
    # Initialize PyChrono
    chrono = pc.Chrono()

    # Define System Parameters
    system_params = {
        "name": "Beam",
        "type": "ANCF",
        "num_elements": 10,  # Number of cable elements
        "cable_length": 1.0, # Length of each cable segment
        "cable_radius": 0.1, # Radius of the cable segments
        "gravity_magnitude": 9.81, # Gravity acceleration
        "gravity_direction": (0, 0, -1) # Gravity vector (pointing downwards)
    }

    # --- 2.  Physical Systems & Objects ---
    # Define the beam elements (cable segments)
    elements = [
        pc.Element("cable",  # Element type: cable
                 cable_length=system_params["cable_length"],
                 cable_radius=system_params["cable_radius"],
                 gravity_direction=system_params["gravity_direction"],
                 mass=1000.0, # Approximate mass of a cable segment (adjust as needed)
                 num_segments=5) # Number of segments per cable element

    ]


    # --- 3.  Simulation Setup ---
    # Define the simulation time
    simulation_time = 10.0  # Simulation duration (seconds)
    chrono.set_simulation_time(simulation_time)

    # --- 4.  Visualization (Irrlicht) ---
    fig, ax = plt.subplots()
    ax.set_xlim(-1.5, 1.5)  # Adjust limits as needed
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.set_title("Beam Simulation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)

    def animate(i):
        #  Create a single element for animation
        element = elements[i]
        x = element.x
        y = element.y
        
        # Update the element's position based on gravity
        element.x += element.gravity_direction[0] * simulation_time
        element.y += element.gravity_direction[1] * simulation_time
        
        # Update the animation
        ax.clear() # Clear the previous frame
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect('equal')
        ax.set_title("Beam Simulation")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True)
        
        # Update the element's position
        element.x = x
        element.y = y
        
        return ax

    ani = animation.FuncAnimation(fig, animate, frames=len(elements), blit=True, repeat=False)
    plt.show()

except Exception as e:
    print(f"Error initializing Chrono: {e}")
    print("Please ensure PyChrono is installed correctly and that your system has the necessary dependencies.")
    print("Consider installing 'pychrono-dependencies' if you encounter issues.")