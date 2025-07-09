import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Create the Chrono system
system = chrono.ChSystemSMC()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.SetWindowSize(1024, 768)
vis.SetNumberOfRenderFrames(50)  # Target FPS
vis.AttachSystem(system)
vis.AddCamera(chrono.ChVectorD(5, 5, 5))
vis.AddTypicalLights()

# Create the Gator vehicle
gator = veh.Gator(system, True, True)  # Visualization and collision enabled
gator.SetContactMethod('SMC')  # Set contact method
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))  # Initial position and orientation
gator.Initialize()

# Create the rigid terrain
terrain = veh.RigidTerrain(system)
terrain.SetPatchDimensions(100, 100)  # Terrain size (x, z)
terrain_texture = chrono.GetChronoDataFile('textures/concrete.jpg')  # Load custom texture
patch = terrain.AddPatch(veh.ChTireMaterial(100000, 0.3, 0.8), chrono.ChCoordsysD(), 100, 100)
patch.material.SetTexture(terrain_texture)
terrain.Initialize()

# Set up TMEASY tire model for all wheels
for wheel in gator.GetWheelIndices():
    tire = gator.GetTire(wheel)
    tire.SetType('TMEASY')
    tire.SetTireRadius(0.3)
    tire.SetTireWidth(0.2)
    tire.SetLongitudinalStiffness(100000)
    tire.SetLateralStiffness(100000)

# Set up interactive driver
driver = veh.ChInteractiveDriver(gator.GetDriverInputs())
driver.SetThrottleSensitivity(0.5)
driver.SetSteeringSensitivity(0.5)
driver.SetBrakingSensitivity(0.5)
driver.EnableKeyboard(True)

# Simulation loop
simulation_time = 0
while vis.Run():
    # Update driver inputs
    driver.Update()
    
    # Advance simulation
    system.DoStepDynamics(1/50)  # Time step for 50 FPS
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Check for exit condition
    if simulation_time > 100:  # Run for 100 seconds
        break
    simulation_time += 1/50

# Close visualization
vis.Close()