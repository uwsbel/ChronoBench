importchrono
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as rob
import pychrono.irrlicht as chronoirr

# Set up the Chrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataFile(''))

# Create a contact material for gears
gear_material = chrono.ChContactMaterialSMC()
gear_material.SetFriction(0.5)
gear_material.SetDampingF(0.2)
gear_material.SetCompliance(0.001)

# Create a visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Create a system with NSC (Non-Smooth Contact)
system = chrono.ChSystemNSC()
vis.AttachSystem(system)

# Create fixed truss
truss = chrono.ChBody()
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetMass(1000)
truss.SetFixed(True)
truss_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(2, 0.2, 2))
truss_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
truss.AddVisualShape(truss_shape)
system.Add(truss)

# Create rotating bar (carrier)
rotating_bar = chrono.ChBody()
rotating_bar.SetPos(chrono.ChVector3d(0, 0.2, 0))
rotating_bar.SetMass(100)
rotating_bar.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
rotating_bar.EnableCollision(True)

# Add motor joint to rotating bar
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rotating_bar, truss, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(2 * math.pi))  # Rotate at 1 RPM
system.Add(motor)

# Create sun gear
sun_gear = chrono.ChBody()
sun_gear.SetPos(chrono.ChVector3d(0, 0.2, 0))
sun_gear.SetMass(50)
sun_gear.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
sun_gear.SetFixed(True)
sun_gear_shape = chrono.ChVisualShapeSphere(0.3)
sun_gear_shape.SetColor(chrono.ChColor(1, 0, 0))
sun_gear.AddVisualShape(sun_gear_shape)
sun_gear.SetMaterialSurface(gear_material)
system.Add(sun_gear)

# Create planet gear
planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVector3d(1, 0.2, 0))
planet_gear.SetMass(50)
planet_gear.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
planet_gear.EnableCollision(True)
planet_gear_shape = chrono.ChVisualShapeSphere(0.3)
planet_gear_shape.SetColor(chrono.ChColor(0, 0, 1))
planet_gear.AddVisualShape(planet_gear_shape)
planet_gear.SetMaterialSurface(gear_material)
system.Add(planet_gear)

# Add revolute joint between sun gear and truss
rev_joint_sun = chrono.ChLinkLockRevolute()
rev_joint_sun.Initialize(sun_gear, truss, chrono.ChCoordsysD(chrono.ChVector3d(0, 0.2, 0), chrono.QUNIT))
system.Add(rev_joint_sun)

# Add revolute joint between planet gear and rotating bar
rev_joint_planet = chrono.ChLinkLockRevolute()
rev_joint_planet.Initialize(planet_gear, rotating_bar, chrono.ChCoordsysD(chrono.ChVector3d(1, 0.2, 0), chrono.QUNIT))
system.Add(rev_joint_planet)

# Add contact between gears
system.AddContact(chrono.ChContactSurface(gear_material, sun_gear.GetCollisionModel(), planet_gear.GetCollisionModel()))

# Run simulation
time = 0
while time < 10:
    # Update motor speed (optional: can vary speed over time)
    motor.GetMotorRotationSpeed().SetSpeedFunction(chrono.ChFunction_Const(2 * math.pi))
    
    # Step the simulation
    system.DoStepDynamics(0.01)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    time += 0.01

# Cleanup visualization
vis.Close()