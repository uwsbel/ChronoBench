import pychrono
import pychrono.physics as pcp
import pychrono.visual as pv
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual import visual_engine

# Simulation parameters
num_gear_motors = 2
fixed_truss_length = 1.0
rotating_bar_length = 0.5
gear_gear_ratio = 1.0
gear_speed = 10.0  # RPM
fixed_truss_mass = 1.0
rotating_bar_mass = 1.0
gear_mass = 0.1
# Define the physical systems
fixed_truss = pcp.FixedTruss(
    fixed_truss_length,
    fixed_truss_mass,
    fixed_truss_position = (0, 0, 0),  # Initial position
    fixed_truss_rotation = 0.0,
    fixed_truss_force = 0.0,
    fixed_truss_interaction = 0.0
)

rotating_bar = pcp.RotatingBar(
    rotating_bar_length,
    rotating_bar_mass,
    rotating_bar_position = (0, 0, 0),
    rotating_bar_rotation = 0.0,
    rotating_bar_force = 0.0,
    rotating_bar_interaction = 0.0
)

gear1_motor = pcp.GearMotor(
    gear_gear_ratio,
    gear_mass,
    gear_position = (0, 0, 0),
    gear_speed = gear_speed,
    gear_interaction = 0.0
)

gear2_motor = pcp.GearMotor(
    gear_gear_ratio,
    gear_mass,
    gear_position = (0, 0, 0),
    gear_speed = gear_speed,
    gear_interaction = 0.0
)

# Simulation setup
simulation = pcp.Simulation(
    fixed_truss,
    rotating_bar,
    gear1_motor,
    gear2_motor,
    fixed_truss_mass,
    rotating_bar_mass,
    gear_mass,
    simulation_duration = 10.0, # seconds
    visual_engine = visual_engine.VisualEngine()
)

# Visualization
simulation.visualize()

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(simulation.time, simulation.position)
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Epicyclic Gear Simulation")
plt.grid(True)
plt.show()