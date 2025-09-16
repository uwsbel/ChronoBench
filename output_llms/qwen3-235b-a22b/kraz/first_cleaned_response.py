import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


kraz = veh.Kraz(system)
kraz.SetInitialPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
kraz.Initialize()


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChVectorD(200, 1, 200), chrono.ChVectorD(0, 0, 0), True)
patch.SetFriction(0.9)
patch.SetRestitution(0.01)
terrain.Initialize()


application = irr.ChIrrApp(system, 'Kraz Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


driver = irr.ChIrrGuiDriver(application.GetDevice())
driver.Initialize()


step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()

time = 0
while application.GetDevice().run():
    
    driver.Synchronize(time)
    kraz.Synchronize(time, driver.GetInputs())
    terrain.Synchronize(time)
    application.Synchronize("Kraz Simulation", driver.GetInputs())

    
    driver.Advance(step_size)
    kraz.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    time += step_size
    realtime_timer.Sleep(step_size)