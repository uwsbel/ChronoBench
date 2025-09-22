import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle








system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = chrono.ChBodyEasyBox(
    200, 20, 200, 1000, True, True, chrono.ChMaterialSurface.NSC
)
terrain.SetPos(chrono.ChVectorD(0, -10, 0))
terrain.SetBodyFixed(True)
system.Add(terrain)


vehicle = chronovehicle.ChVehicle("Kraz")
vehicle.SetSystem(system)
vehicle.Initialize()


vehicle.SetPos(chrono.ChVectorD(0, 0, 0))
vehicle.SetRot(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1)))


driver = chronovehicle.ChDriver(vehicle)








application = chronoirr.ChIrrApp(system, "Kraz Simulation", chronoirr.dimension2du(1280, 720))
application.AddTypicalSky()
application.AddTypicalLights()


camera = application.GetSceneManager().getActiveCamera()
camera.setPosition(chrono.ChVectorD(10, 5, -10))
camera.setTarget(chrono.ChVectorD(0, 0, 0))







while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    driver.Synchronize(vehicle.GetChasisBody())

    
    system.DoStepDynamics(application.GetTimeStep())

    application.EndScene()