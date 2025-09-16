import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.Init()


initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.KrAZ(initLoc, initRot)


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_C_PI / 2)))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)


driver = veh.ChDriver(vehicle.GetVehicle())


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), chrono.VECT_Y)
vis.SetCameraVertical(chrono.VECT_Y)
vis.EnableShadows()
vis.AttachLight(chrono.ChVectorD(100, 100, 100), 100, chrono.ChColor(1, 1, 1))
vis.AttachLight(chrono.ChVectorD(-100, -100, 100), 100, chrono.ChColor(1, 1, 1))
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize("Kraz Vehicle Simulation", driver_inputs)
    vehicle.Advance(0.01)
    terrain.Advance(0.01)
    vis.Advance(0.01)