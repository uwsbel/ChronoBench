import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetInitFwdVel(5.0)
vehicle.SetTireType(veh.TireType_TMEasy)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.0, 0, 30, 1000, 2e5, 0.9, 0.1, 0.9, 0)
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize("terrain_heightmap.bmp", 0.1, 0, 0.1, 100, 100)


driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetCameraProperties(chrono.ChVector3d(10, 10, 5), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1))
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.EnableShadows(True)
vis.AttachVehicle(vehicle.GetVehicle())


light = irr.ChLight(irr.ChLight_Type(0))
light.SetLightType(irr.LIGHT_DIRECTIONAL)
light.SetDirection(chrono.ChVector3d(-1, -1, -1))
vis.AddLight(light)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize("HMMWV on SCM Deformable Terrain", driver_inputs)
    vehicle.Advance(1e-3)
    terrain.Advance(1e-3)
    vis.Advance(1e-3)
    vis.Run()