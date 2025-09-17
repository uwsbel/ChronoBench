import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.physics import Physics
from pychrono.simulation import Simulation

# --- 1. Initialization and Core Components ---
try:
    physics = Physics()
    simulation = Simulation(physics)
    simulation.set_initial_state(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])) # Initial position and orientation
    simulation.set_solver(pychrono.solver.Euler)
    simulation.set_timestepper(simulation.get_time())

    print("PyChrono simulation initialized successfully.")

except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()


# --- 2. Physical Systems and Objects ---
# Define the beam elements
def create_beam_element(length, cross_section_area, yield_strength):
    """Creates a simple beam element for finite element analysis."""
    return np.array([cross_section_area, 0.0, 0.0], dtype=np.float64)

# Create the beam elements
beam_elements = [create_beam_element(1.0, 0.5, 10e-6) for _ in range(3)]

# Define the supports
def create_support(length, cross_section_area, yield_strength):
    """Creates a support for the beam."""
    return np.array([cross_section_area, 0.0, 0.0])

# --- 3. Custom Motor Functions ---
def motor_a(time, force):
    """Simulates a motor force applied to the beam."""
    return force

def motor_b(time, force):
    """Simulates a motor force applied to the beam."""
    return force

# --- 4. Visualization ---
def visualize(time, simulation):
    """Visualizes the simulation using Irrlicht."""
    plt.figure(figsize=(10, 6))
    plt.plot(time, beam_elements[:, 0], label="Beam 1")
    plt.plot(time, beam_elements[:, 1], label="Beam 2")
    plt.xlabel("Time")
    plt.ylabel("Length")
    plt.title("Beam Buckling Simulation")
    plt.legend()
    plt.grid(True)
    plt.show()

# --- 5. Main Simulation Loop ---
if __name__ == "__main__":
    try:
        simulation.run(time=simulation.get_time(),  # Run the simulation
                        steps=100) # Number of time steps
        print("Simulation completed.")
        visualize(simulation.get_time(), simulation)
    except Exception as e:
        print(f"Simulation failed: {e}")