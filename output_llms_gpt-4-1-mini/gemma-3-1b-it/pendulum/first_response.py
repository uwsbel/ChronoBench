import pychrono
import pychrono.core as chrono
import pychrono.visuals as visuals
import pychrono.events as events
import numpy as np

# --- Simulation Parameters ---
gravity = 9.81  # m/s^2
initial_angle = 30.0  # degrees
initial_angular_velocity = 0.0
initial_position = 0.0
initial_velocity = 0.0
simulation_duration = 10.0  # seconds
num_steps = 100

# --- Physical Systems ---
mass = 1.0  # kg
length = 1.0  # meters
joint_length = 0.5  # meters
# --- Ground Body ---
ground_body_position = 0.0
ground_body_velocity = 0.0

# --- Pendulum ---
pendulum_mass = mass
pendulum_inertia = 1.0  # kg (Assume mass is 1 kg)
pendulum_angle = initial_angle
pendulum_angular_velocity = initial_angular_velocity

# --- Revolute Joint ---
joint_angle = 0.0
joint_radius = 0.05 # meters (adjust for joint size)
joint_length = 0.5

# --- Initial Conditions ---
# Initial position and velocity
pendulum_position = pendulum_position
pendulum_velocity = pendulum_angular_velocity

# --- Simulation Loop ---
for i in range(simulation_duration):
    # 1. Update Position and Velocity
    pendulum_position += pendulum_angular_velocity * i
    pendulum_velocity += pendulum_angular_velocity * i

    # 2. Apply Forces (Simplified - can be expanded)
    #   This is a placeholder.  In a real simulation, you'd need to
    #   model the forces acting on the pendulum (e.g., gravity, friction).
    #   For this example, we'll just simulate a constant force.
    force = 0.0  # Force in Newtons
    force = force * (pendulum_mass / length) # Force proportional to mass/length
    force = force * (pendulum_inertia / length) # Force proportional to inertia/length
    force = force * (1.0 - np.cos(pendulum_angle)) #  Simulate a constant, downward force
    pendulum_position += force * (pendulum_mass / length)

    # 3. Update Joint Angle
    joint_angle += joint_angle * 0.1  # Small change per step

    # 4.  Visualize (Simple)
    visuals.plot(pendulum_position, pendulum_velocity,  
                label="Pendulum Position", color="blue")
    visuals.plot(joint_angle, joint_radius, color="red")
    visuals.plot(pendulum_angle, pendulum_inertia, color="green")

    # 5. Log Data (Periodic)
    if i % 10 == 0:
        print(f"Step {i}: Pendulum Position = {pendulum_position:.2f}, Velocity = {pendulum_velocity:.2f}")

    # 6.  Event Handling (Optional - for more complex simulations)
    #   This is a very basic example - you would likely want to
    #   handle events like collisions or changes in the system.
    #   For now, we just log the position.
    #   events.log_event(events.Pendulum_Position, "Pendulum Position", i)


# --- End of Simulation ---
visuals.plot(pendulum_position, pendulum_velocity,  
                label="Pendulum Position", color="blue")
visuals.plot(joint_angle, joint_radius, color="red")
visuals.plot(pendulum_angle, pendulum_inertia, color="green")
visuals.plot(pendulum_angle, pendulum_mass, color="yellow") # add mass for visual clarity
visuals.plot(pendulum_mass, 0.0, color="black") # Add mass for visual clarity
visuals.plot(pendulum_mass, 0.0, color="black") # Add mass for visual clarity

visuals.plot(pendulum_angle, 0.0, color="red") # Add zero to the plot for better visibility
visuals.plot(pendulum_angle, 0.0, color="red") # Add zero to the plot for better visibility

# ---  End of Simulation ---
print("Simulation complete.")