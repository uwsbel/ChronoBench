import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the PyChrono environment
chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')

# Create the physical system
system = chrono.ChSystemNSC()

# Set default parameters
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity in Y direction

# Create the ground body (fixed)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)

# Add visual shape for ground (platform)
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, 0.1, 1)
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVectorD(0, -0.1, 0)))

# Set ground material properties for visualization
ground_material = chrono.ChVisualMaterial()
ground_material.SetDiffuseColor(chrono.ChColor(0.3, 0.3, 0.3))
ground_shape.SetMaterial(0, ground_material)

# Create the mass body
mass_body = chrono.ChBody()
mass_body.SetMass(1.0)  # 1 kg mass
mass_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  # Inertia tensor
mass_body.SetPos(chrono.ChVectorD(0, 2, 0))  # Initial position 2m above ground
mass_body.SetPos_dt(chrono.ChVectorD(0, 0, 0))  # Initial velocity
mass_body.SetBodyFixed(False)
system.Add(mass_body)

# Add visual shape for mass (sphere)
mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.2
mass_body.AddVisualShape(mass_shape)

# Set mass material properties for visualization
mass_material = chrono.ChVisualMaterial()
mass_material.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))
mass_shape.SetMaterial(0, mass_material)

# Create the spring-damper connection using ChLinkTSDA
spring_damper = chrono.ChLinkTSDA()

# Set connection points
# Point on ground (origin)
ground_point = chrono.ChVectorD(0, 0, 0)
# Point on mass body (center)
mass_point = chrono.ChVectorD(0, 0, 0)

# Initialize the spring-damper link
spring_damper.Initialize(ground, mass_body, False, ground_point, mass_point)

# Set spring-damper parameters
rest_length = 1.5  # Rest length of spring (m)
spring_constant = 50.0  # Spring stiffness (N/m)
damping_coefficient = 2.0  # Damping coefficient (N*s/m)

spring_damper.SetRestLength(rest_length)
spring_damper.SetSpringCoefficient(spring_constant)
spring_damper.SetDampingCoefficient(damping_coefficient)

# Add the spring-damper to the system
system.Add(spring_damper)

# Create visual representation of the spring
spring_visual = chrono.ChVisualShapeSegment()
spring_damper.AddVisualShape(spring_visual)

# Set spring visual properties
spring_visual_material = chrono.ChVisualMaterial()
spring_visual_material.SetDiffuseColor(chrono.ChColor(0.2, 0.8, 0.2))
spring_visual.SetMaterial(0, spring_visual_material)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(4, 3, 4), chrono.ChVectorD(0, 1, 0))

# Add lighting
vis.AddTypicalLights()

# Set up the simulation parameters
time_step = 0.01
simulation_time = 10.0
current_time = 0

# Simulation loop
while current_time < simulation_time:
    # Update the visualization
    if not vis.Run():
        break
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    
    # Display some information
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Time: {current_time:.2f}s",
        chronoirr.irr.core.rect(10, 10, 200, 30),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    mass_pos = mass_body.GetPos()
    spring_length = spring_damper.GetLength()
    spring_force = spring_damper.GetSpringForce()
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Mass Y-pos: {mass_pos.y:.3f}m",
        chronoirr.irr.core.rect(10, 30, 250, 50),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Spring Length: {spring_length:.3f}m",
        chronoirr.irr.core.rect(10, 50, 250, 70),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Spring Force: {spring_force:.3f}N",
        chronoirr.irr.core.rect(10, 70, 250, 90),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    vis.EndScene()
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    current_time += time_step

# Clean up
vis.GetDevice().closeDevice()