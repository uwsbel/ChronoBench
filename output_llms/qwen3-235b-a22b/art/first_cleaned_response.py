import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irrlicht


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = irrlicht.ChIrrApp(my_system, 'ARTcar Simulation', irrlicht.dimension2du(800, 600))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


terrain = veh.RigidTerrain(my_system)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), 
                         chrono.ChVectorD(0, 0, 0), 
                         chrono.ChVectorD(200, 200, 1), 
                         True)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


artcar = veh.ARTcar(my_system)
artcar.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), 
                                    chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
artcar.SetContactMethod(chrono.ChMaterialSurfaceNSC())
artcar.SetChassisVisualizationType(veh.VisualizationType_MESH)
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChInteractiveDriverIRR(application.GetDevice(), artcar, "Driver Controls")
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()


while application.GetDevice().run():
    time = my_system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    artcar.Synchronize(time, driver.GetInputs(), terrain)
    
    
    my_system.DoStepDynamics(0.02)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()