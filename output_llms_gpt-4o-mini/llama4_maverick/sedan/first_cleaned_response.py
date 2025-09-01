import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


print("Copyright (c) 2023 Project Chrono")


vehicle = veh.WheeledVehicle(chrono.GetChOutputPath() + "BMW_E90_sedan", veh.ChContactMethod_SMC)


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(-math.pi / 2)), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))
vehicle.Initialize()


tireParams = veh.TMeasyTire::GetTMeasyParams(28 * 0.9)
left_front_tire = veh.TMeasyTire(vehicle.GetLeftFrontWheel(), tireParams)
right_front_tire = veh.TMeasyTire(vehicle.GetRightFrontWheel(), tireParams)
left_rear_tire = veh.TMeasyTire(vehicle.GetLeftRearWheel(), tireParams)
right_rear_tire = veh.TMeasyTire(vehicle.GetRightRearWheel(), tireParams)
vehicle.GetLeftFrontWheel().SetTire(left_front_tire)
vehicle.GetRightFrontWheel().SetTire(right_front_tire)
vehicle.GetLeftRearWheel().SetTire(left_rear_tire)
vehicle.GetRightRearWheel().SetTire(right_rear_tire)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('BMW E90 Sedan Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.DoStepDynamics(1e-3)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    driver.Synchronize(time)
    driver.Advance(1e-3)


vis.Destroy()