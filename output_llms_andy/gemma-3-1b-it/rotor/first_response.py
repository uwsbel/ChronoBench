import pychrono
import pychrono.core as pc
import pychrono.visual as pv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. Initialize PyChrono Environment
try:
    chrono = pc.chrono.Chrono()
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()

# 2. Define Physical Systems and Objects
# Jeffcott Rotor
rotor_model = pc.Model("JeffcottRotor")
rotor_model.set_mass(1000)  # kg
rotor_model.set_length(1.0)  # meters
rotor_model.set_rotation_speed(0.1) # rad/s

# Flywheel
flywheel_model = pc.Model("Flywheel")
flywheel_model.set_mass(1000)  # kg
flywheel_model.set_length(0.5)  # meters
flywheel_model.set_rotation_speed(0.05) # rad/s

# Simulation Parameters
simulation_time = 10  # seconds
time_step = 0.01  # seconds
num_steps = int(simulation_time / time_step)

# 3. Create Beam
beam = pc.Beam(
    model=rotor_model,
    length=1.0,  # meters
    mass=1000, # kg
    rotation_speed=rotor_model.set_rotation_speed(),
    # Add a simple support to the beam (e.g., a simple cylinder)
    support_radius=0.1,
    support_height=0.5
)

# 4. Attach Flywheel to Beam
flywheel_position = 0.0  # meters
flywheel_location = beam.get_position()
flywheel_model.set_position(flywheel_location + flywheel_position)

# 5. Create Motor
motor = pc.Motor(
    model=pc.Motor("Motor"),
    speed=rotor_model.set_rotation_speed(),
    # Set a simple damping factor to simulate friction
    damping=0.1
)

# 6. Simulation Loop
for i in range(num_steps):
    # Update Motor Position
    motor.set_position(motor.get_position() + motor.get_speed() * time_step)

    # Apply forces
    rotor_model.set_force(flywheel_model, 0.0)  # Apply a force to the flywheel
    motor.set_force(0.0, 0.0)  # Apply a force to the motor

    # Update beam position
    beam.set_position(beam.get_position() + time_step)

    # Visualization
    pv.plot(beam, title="Jeffcott Rotor Beam Dynamics")
    pv.show(beam)

    # Print Simulation Information
    print(f"Step {i+1}: Time = {time_step:.2f} s")

# 7. Cleanup
chrono.close()