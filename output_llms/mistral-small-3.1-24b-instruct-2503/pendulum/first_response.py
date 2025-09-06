import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the Chrono::Engine
chrono.SetChronoDataPath('../data/')

# Create a Chrono::Engine system
system = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
system.AddBody(ground)

# Create the pendulum body
pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))  # Initial position at (0, 1, 0)
pendulum.SetMass(1.0)  # Mass of the pendulum
pendulum.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))  # Inertia tensor
pendulum.SetCollide(True)
system.AddBody(pendulum)

# Create a revolute joint to connect the pendulum to the ground
revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(revolute)

# Create an Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))

# Set up the simulation step size and total simulation time
step_size = 0.01
total_time = 10.0

# Initialize simulation time and logging variables
simulation_time = 0.0
log_interval = 0.1
log_time = 0.0

# Main simulation loop
while simulation_time < total_time:
    system.DoStepDynamics(step_size)
    simulation_time += step_size

    # Log the pendulum's position and velocity
    if simulation_time >= log_time:
        pendulum_pos = pendulum.GetPos()
        pendulum_vel = pendulum.GetPos_dt()
        print(f"Time: {simulation_time:.2f}, Position: {pendulum_pos}, Velocity: {pendulum_vel}")
        log_time += log_interval

    # Update the visualization
    vis.Run()

# Close the visualization system
vis.Close()