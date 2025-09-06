import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('path_to_chrono_data')  # Set path to Chrono data directory

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add the required physical systems and objects

# Create a fixed truss (ground body)
truss = chrono.ChBodyEasyBox(2, 0.2, 0.2, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
system.Add(truss)

# Create a rotating bar (carrier)
bar_length = 1.0
bar = chrono.ChBodyEasyBox(bar_length, 0.1, 0.1, 10, True, True)
bar.SetPos(chrono.ChVectorD(0, 0.5, 0))
bar.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  # No initial rotation
system.Add(bar)

# Create the sun gear (fixed to ground)
sun_gear = chrono.ChBodyEasyCylinder(0.2, 0.1, 1000, True, True)
sun_gear.SetPos(chrono.ChVectorD(0, 0.5, 0))
sun_gear.SetBodyFixed(True)
system.Add(sun_gear)

# Create the planet gear
planet_gear = chrono.ChBodyEasyCylinder(0.1, 0.1, 5, True, True)
planet_gear.SetPos(chrono.ChVectorD(0.5, 0.5, 0))
system.Add(planet_gear)

# 3. Set necessary default parameters such as positions, forces, and interactions

# Create a revolute joint between the bar and ground (to allow rotation)
revolute_bar = chrono.ChLinkLockRevolute()
revolute_bar.Initialize(bar, truss, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)))
system.Add(revolute_bar)

# Create a gear between the sun gear and planet gear
gear_ratio = 2.0  # Sun gear is twice as large as planet gear
gear = chrono.ChLinkGear()
gear.Initialize(sun_gear, planet_gear, chrono.ChCoordsysD(), chrono.ChCoordsysD(), gear_ratio)
system.Add(gear)

# Create a revolute joint between the planet gear and the bar
revolute_planet = chrono.ChLinkLockRevolute()
revolute_planet.Initialize(planet_gear, bar, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0.5, 0)))
system.Add(revolute_planet)

# Add a motor to enforce constant rotation speed on the bar
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
motor.SetMotorFunction(chrono.ChFunction_Const(1.0))  # 1 rad/s constant speed
system.Add(motor)

# 4. Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1, 2), chrono.ChVectorD(0, 0.5, 0))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0), 10, chrono.ChColor(1, 1, 1))

# Set custom materials for better visualization
truss_material = chrono.ChVisualMaterial()
truss_material.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.5))
truss.GetVisualModel(0).SetMaterial(0, truss_material)

bar_material = chrono.ChVisualMaterial()
bar_material.SetDiffuseColor(chrono.ChColor(0.2, 0.4, 0.8))
bar.GetVisualModel(0).SetMaterial(0, bar_material)

sun_gear_material = chrono.ChVisualMaterial()
sun_gear_material.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))
sun_gear.GetVisualModel(0).SetMaterial(0, sun_gear_material)

planet_gear_material = chrono.ChVisualMaterial()
planet_gear_material.SetDiffuseColor(chrono.ChColor(0.2, 0.8, 0.2))
planet_gear.GetVisualModel(0).SetMaterial(0, planet_gear_material)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(0.01)  # Step size of 0.01 seconds