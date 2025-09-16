import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.ChRigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(200, 100, 1))
terrain.Initialize()


vehicle = veh.M113(system)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0)))
vehicle.Initialize()


driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


application = chronoirr.ChIrrApp(system, "M113 Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, -5, 0.5), chronoirr.vector3df(0, 0, 0))


step_size = 1e-3
step_timer = chrono.ChRealtimeStepTimer()


while application.GetDevice().run():
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    system.DoStepDynamics(step_size)
    
    
    step_timer.Spin(step_size)