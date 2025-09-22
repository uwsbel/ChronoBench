import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.physics import Physics
from pychrono.simulation import Simulation


try:
    physics = Physics()
    simulation = Simulation(physics)
    simulation.set_initial_state(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])) 
    simulation.set_solver(pychrono.solver.Euler)
    simulation.set_timestepper(simulation.get_time())

    print("PyChrono simulation initialized successfully.")

except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()




def create_beam_element(length, cross_section_area, yield_strength):
    
    return np.array([cross_section_area, 0.0, 0.0], dtype=np.float64)


beam_elements = [create_beam_element(1.0, 0.5, 10e-6) for _ in range(3)]


def create_support(length, cross_section_area, yield_strength):
    
    return np.array([cross_section_area, 0.0, 0.0])


def motor_a(time, force):
    
    return force

def motor_b(time, force):
    
    return force


def visualize(time, simulation):
    
    plt.figure(figsize=(10, 6))
    plt.plot(time, beam_elements[:, 0], label="Beam 1")
    plt.plot(time, beam_elements[:, 1], label="Beam 2")
    plt.xlabel("Time")
    plt.ylabel("Length")
    plt.title("Beam Buckling Simulation")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    try:
        simulation.run(time=simulation.get_time(),  
                        steps=100) 
        print("Simulation completed.")
        visualize(simulation.get_time(), simulation)
    except Exception as e:
        print(f"Simulation failed: {e}")