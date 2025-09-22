import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("chrono_data/")


system = chrono.ChSystemSMC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAI_BORWEIN)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("CityBus Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.SetCameraPosition(chrono.ChVector3d(5, 5, 5))
vis.SetCameraRotation(chrono.ChVector3d(-45, 45, 0))


bus = veh.CityBus()
bus.SetContactMethod(veh.VehicleContactMethod.SMC)
bus.SetChassisCollisionType(veh.VehicleCollisionType.BOX)
bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
bus.Initialize()


terrain = veh.RigidTerrain(bus.GetSystem())
terrain_texture = chrono.GetChronoDataFile("textures/concrete.jpg")
terrain_patch = terrain.AddPatch(terrain_texture, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.Driver()
driver.Initialize(bus.GetDriverInputs())


driver_input = bus.GetDriverInputs()
driver_input.SetThrottle(0)
driver_input.SetSteering(0)
driver_input.SetBraking(0)


camera = vis.GetCamera()
camera.SetDistance(10)
camera.SetRotation(chrono.ChVector3d(-45, 45, 0))


simulation_time = 0
render_step = 1 / 50  

while vis.Run():
    
    driver.Synchronize(time=render_step)
    driver_input.SetThrottle(0.5)  
    
    
    system.DoStepDynamics(render_step)
    simulation_time += render_step
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if vis.KeyHit(chrono.CH_KEY_ESCAPE):
        break


vis.Close()