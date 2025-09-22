import PyChrono as chrono
import PyChrono.Irrlicht as chronoirr
import PyChrono.Vehicle as vehicle
import PyChrono.Postprocess as postprocess


chrono.Init_Chrono()


vehicle_system = vehicle.ChVehicle()
vehicle_system.Initialize(chrono.GetChronoDataFile('vehicle/hmmwv/hmmwv.tire'), chrono.GetChronoDataFile('vehicle/hmmwv/hmmwv.chassis'))


vehicle_system.SetChassisPosition(chrono.ChVectorD(0, 0, 0.5))
vehicle_system.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))


vehicle_system.SetContactMethod(chrono.ChContactMethod_NSC)


vehicle_system.SetChassisVisualizationType(chrono.ChVehicleVisualizationType_PRIMITIVES)


terrain = chrono.ChRigidTerrain(vehicle_system.GetSystem())
terrain.SetTexture(chrono.GetChronoDataFile('terrain/textures/tarmac.jpg'))
terrain.SetContactMaterial(3e7, 0.4)
terrain.Initialize(chrono.ChVectorD(-100, 0, -100), chrono.ChVectorD(100, 0, 100))


terrain.Add(vehicle_system.GetChassis())


driver = vehicle.ChIrrGuiDriver(vehicle_system, 0.01)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.2)
driver.SetBrakingDelta(-0.2)


application = chronoirr.ChIrrApp(vehicle_system, "ARTcar Simulation", chronoirr.dimension2d(1280, 720), driver)
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 5, -10))
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddShadowAll()


postprocessor = postprocess.ChChronoPostprocess(vehicle_system)
postprocessor.SetVerbose(True)
postprocessor.AddTypicalLoggers(vehicle_system)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

chrono.Chrono_Close()