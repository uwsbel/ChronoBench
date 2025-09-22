import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set up the environment
chrono.SetChronoDataPath("chrono_data/")

# Create the system
system = chrono.ChSystemSMC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAI_BORWEIN)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)

# Create visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("CityBus Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.SetCameraPosition(chrono.ChVector3d(5, 5, 5))
vis.SetCameraRotation(chrono.ChVector3d(-45, 45, 0))

# Create the vehicle
bus = veh.CityBus()
bus.SetContactMethod(veh.VehicleContactMethod.SMC)
bus.SetChassisCollisionType(veh.VehicleCollisionType.BOX)
bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
bus.Initialize()

# Create the terrain
terrain = veh.RigidTerrain(bus.GetSystem())
terrain_texture = chrono.GetChronoDataFile("textures/concrete.jpg")
terrain_patch = terrain.AddPatch(terrain_texture, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Create driver system
driver = veh.Driver()
driver.Initialize(bus.GetDriverInputs())

# Set up driver controls
driver_input = bus.GetDriverInputs()
driver_input.SetThrottle(0)
driver_input.SetSteering(0)
driver_input.SetBraking(0)

# Set up camera to follow the vehicle
camera = vis.GetCamera()
camera.SetDistance(10)
camera.SetRotation(chrono.ChVector3d(-45, 45, 0))

# Set up simulation loop
simulation_time = 0
render_step = 1 / 50  # 50 FPS

while vis.Run():
    # Update driver inputs
    driver.Synchronize(time=render_step)
    driver_input.SetThrottle(0.5)  # Example: constant throttle
    
    # Update simulation
    system.DoStepDynamics(render_step)
    simulation_time += render_step
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Check for exit condition
    if vis.KeyHit(chrono.CH_KEY_ESCAPE):
        break

# Cleanup
vis.Close()