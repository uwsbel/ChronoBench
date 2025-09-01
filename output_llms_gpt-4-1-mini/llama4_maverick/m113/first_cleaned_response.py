import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


print("Copyright (c) 2023 projectchrono.org")


vehicle = veh.M113(chrono.ChContactMethod_SMC)
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-5, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetInitFwdVel(0)
vehicle.SetInitWheelAngVel(chrono.ChVectorD(0))
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Vehicle Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(driver_inputs)
    terrain.Advance(time)
    driver.Advance(time)
    vis.Advance(time)
    vis.Render()
    realtime_timer.Spin(1.0 / 60.0)