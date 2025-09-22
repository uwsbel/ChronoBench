import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the ChSystem
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)  # Fix the ground so it doesn't move
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.8))
system.Add(ground)

# Create the pendulum body
pendulum_length = 4.0
pendulum_radius = 0.2
pendulum_mass = 1.0

# Create a cylinder for the pendulum rod
pendulum = chrono.ChBodyEasyCylinder(
    pendulum_radius,  # radius
    pendulum_length,  # height
    1000,             # density
    True,             # visualization?
    True              # collision?
)
pendulum.SetPos(chrono.ChVectorD(0, -2, 0))  # Position the pendulum
pendulum.SetMass(pendulum_mass)
pendulum.GetVisualShape(0).SetColor(chrono.ChColor(0.75, 0.1, 0.1))

# Calculate and set inertia for the pendulum
inertia_xx = pendulum_mass * (3 * pendulum_radius**2 + pendulum_length**2) / 12
inertia_yy = pendulum_mass * pendulum_radius**2 / 2
inertia_zz = inertia_xx
pendulum.SetInertiaXX(chrono.ChVectorD(inertia_xx, inertia_yy, inertia_zz))

# Set an initial angular velocity to start the pendulum motion
pendulum.SetWvel_par(chrono.ChVectorD(0, 0, 1.0))

system.Add(pendulum)

# Create a revolute joint between the pendulum and ground
joint_pos = chrono.ChVectorD(0, 0, 0)  # Position of the joint

# Create the joint
revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(pendulum, ground, 
                          chrono.ChCoordsysD(joint_pos, 
                                            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0))))
system.Add(revolute_joint)

# Create the visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Simulation parameters
time_step = 0.01
log_interval = 0.1
total_time = 10.0

# For logging
last_log_time = 0.0

# Start the simulation
while vis.Run() and system.GetChTime() < total_time:
    # Update the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Step the simulation
    system.DoStepDynamics(time_step)
    
    # Log the pendulum's position and velocity periodically
    if system.GetChTime() - last_log_time >= log_interval:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        ang_vel = pendulum.GetWvel_loc()
        
        print(f"Time: {system.GetChTime():.2f} s")
        print(f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) m")
        print(f"Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f}) m/s")
        print(f"Angular velocity: ({ang_vel.x:.2f}, {ang_vel.y:.2f}, {ang_vel.z:.2f}) rad/s")
        print("------------------------------------------------")
        
        last_log_time = system.GetChTime()

print("Simulation completed!")