import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print('Copyright (c) 2023 Project Chrono')


vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetInitFwdVel(5)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChVector3d(20, 20, 1))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterial(vehicle.GetSystem().GetContactMethod())
terrain.Initialize()


driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, -5, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetVehicle())


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize('', driver_inputs)
    vehicle.Advance(1e-3)
    terrain.Advance(1e-3)
    vis.Advance(1e-3)
    driver.Advance(1e-3)
    vis.Run()