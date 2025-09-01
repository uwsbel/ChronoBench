import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')


my_system = chrono.ChSystemNSC()


vehicle = chrono.vehicle.ChVehicle()
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetTireType(chrono.TireModelType_RIGID)


my_system.Add(vehicle)


terrain = chrono.vehicle.RigidTerrain(vehicle.GetSystem())
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(1e7, 0.3)
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/rock.jpg"))


my_system.Add(terrain)


driver = chrono.vehicle.ChDriver(vehicle)
driver.Initialize()


application = irr.ChIrrApp(vehicle, 'FEDA Vehicle Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(0, 3, -6))
application.SetTimestep(0.02)


vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    
    driver.Synchronize(my_system.GetChTime())
    vehicle.Synchronize(my_system.GetChTime(), driver.GetInputs())
    my_system.DoStepDynamics(0.02)