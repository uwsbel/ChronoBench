import pychrono
import pychrono.events as events
import pychrono.visuals as visuals
import pychrono.utils as utils
import time
import random

# --- Simulation Parameters ---
simulation_duration = 60  # Seconds
initial_position = (0, 0, 0)  # Initial position of the floor
initial_speed = 0.1  # Initial angular speed of the crankshaft
motor_speed = 5.0  # Motor speed (radians/second)
friction_coefficient = 0.8  # Friction coefficient for the joints
joint_strength = 1000  # Joint strength (N) - adjust as needed
cylinder_radius = 0.1  # Radius of the piston
cylinder_height = 0.5 # Height of the piston
# --- System Setup ---
# Create the floor (truss)
floor = visuals.Floor(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,  # Mass of the floor (kg) - adjust as needed
    friction=0.1
)

# Create the crankshaft
crankshaft = visuals.Crankshaft(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,
    friction=0.8,
    speed=initial_speed,
    rotation_speed=motor_speed
)

# Create the connecting rod
connecting_rod = visuals.ConnectingRod(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,
    friction=0.1
)

# Create the piston
piston = visuals.Piston(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,
    friction=0.1,
    height=cylinder_height,
    speed=initial_speed
)

# --- Initial State ---
# Set initial positions and speeds
floor.x = initial_position[0]
floor.y = initial_position[1]
floor.z = initial_position[2]
crankshaft.x = initial_position[0]
crankshaft.y = initial_position[1]
crankshaft.z = initial_position[2]
connecting_rod.x = initial_position[0]
connecting_rod.y = initial_position[1]
connecting_rod.z = initial_position[2]
piston.x = initial_position[0]
piston.y = initial_position[1]
piston.z = initial_position[2]


# --- Simulation Loop ---
for i in range(simulation_duration):
    # Update the position of the floor
    floor.x += initial_speed * i
    floor.y += initial_speed * i
    floor.z += initial_speed * i

    # Update the position of the crankshaft
    crankshaft.x += initial_speed * i
    crankshaft.y += initial_speed * i
    crankshaft.z += initial_speed * i

    # Update the position of the connecting rod
    connecting_rod.x += initial_speed * i
    connecting_rod.y += initial_speed * i
    connecting_rod.z += initial_speed * i

    # Update the position of the piston
    piston.x += initial_speed * i
    piston.y += initial_speed * i
    piston.z += initial_speed * i

    # Apply friction
    floor.friction += joint_strength * (i / simulation_duration)
    crankshaft.friction += joint_strength * (i / simulation_duration)
    connecting_rod.friction += joint_strength * (i / simulation_duration)
    piston.friction += joint_strength * (i / simulation_duration)

    # Simulate the motor
    if motor_speed > 0:
        crankshaft.speed = motor_speed
        connecting_rod.speed = motor_speed
        piston.speed = motor_speed
    else:
        crankshaft.speed = 0
        connecting_rod.speed = 0
        piston.speed = 0

    # Visualize the simulation
    visuals.Draw(
        floor,
        x=floor.x,
        y=floor.y,
        z=floor.z,
        color=(1, 0, 0),  # Red
        linewidth=2
    )

    visuals.Draw(
        crankshaft,
        x=crankshaft.x,
        y=crankshaft.y,
        z=crankshaft.z,
        color=(0, 0, 1),  # Blue
        linewidth=2
    )

    visuals.Draw(
        connecting_rod,
        x=connecting_rod.x,
        y=connecting_rod.y,
        z=connecting_rod.z,
        color=(0, 0, 1),  # Blue
        linewidth=2
    )

    visuals.Draw(
        piston,
        x=piston.x,
        y=piston.y,
        z=piston.z,
        color=(1, 1, 0),  # Green
        linewidth=2
    )

    time.sleep(0.01)  # Adjust for simulation speed

print("Simulation complete.")