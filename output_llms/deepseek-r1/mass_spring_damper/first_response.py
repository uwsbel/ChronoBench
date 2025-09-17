#!/usr/bin/env python3
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. Initialize system and core components
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity

# 2. Create physical bodies
# Ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# Mass body (free to move)
mass = chrono.ChBody()
mass.SetMass(1.0)  # 1 kg
mass.SetPos(chrono.ChVectorD(0, 1, 0))  # Initial position
mass.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
system.Add(mass)

# 3. Add visualization shapes
# Ground visualization (box)
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.1, 0.5)
ground.AddVisualShape(ground_shape)

# Mass visualization (sphere)
mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.2
mass.AddVisualShape(mass_shape)

# 4. Create spring-damper between ground and mass
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
spring.SetRestLength(1.0)  # Rest length matches initial distance
spring.SetSpringCoefficient(100)  # N/m
spring.SetDampingCoefficient(1)  # N·s/m

# Spring visualization
spring_vis = chrono.ChLinkTSDA()
spring_vis = spring.AddVisualShape(chrono.ChSpringShape(0.02, 80, 15))
system.Add(spring)

# 5. Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1, 3))
vis.AddTypicalLights()

# 6. Simulation loop
timestep = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)