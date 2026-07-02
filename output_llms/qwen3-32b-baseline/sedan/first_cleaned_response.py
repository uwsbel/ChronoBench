import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


bmw = vehicle.BMW_E90(sys)
bmw.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
bmw.SetChassisVisualizationType(vehicle.VisualizationType.MESH)
bmw.SetSuspensionVisualizationType(vehicle.VisualizationType.PRIMITIVES)
bmw.SetSteeringVisualizationType(vehicle.VisualizationType.PRIMITIVES)
bmw.SetWheelVisualizationType(vehicle.VisualizationType.NONE)
bmw.Initialize()


for name, wheel in bmw.GetWheelList():
    tire = vehicle.TMeasyTire(sys)
    tire.SetVisualizationType(vehicle.VisualizationType.MESH)
    tire.Initialize()
    bmw.AddTire(tire, wheel)


terrain = vehicle.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(100, 1, 100))
patch.SetTexture(chrono.GetChronoDataFile('terrain/textures/concrete.jpg'), 100, 100)
patch.AddLogo(chrono.GetChronoDataFile('terrain/textures/logo.png'), chrono.ChVectorD(0, 0, 0), 10, 10)
terrain.Initialize()


application = irr.ChIrrApp(sys, 'BMW E90 Simulation', irr.dimension2d_u32(1280, 720))
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 2, -6), irr.vector3df(0, 1.5, 0))
application.AddTypicalLights()
application.SetShowInfos(True)
application.SetChaseCamera(chrono.ChVectorD(0, 0.5, -1.5), 6.0, 0.5)


driver = vehicle.ChInteractiveDriverIRR(application, bmw.GetSteering(), bmw.GetSuspensionFrontLeft().GetSpindle(), bmw.GetSuspensionRearLeft().GetSpindle())
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.Initialize()


application.SetTimestep(0.02)

while application.GetDevice().run():
    driver.Update()
    sys.DoStepDynamics(application.GetTimeStep())
    application.BeginScene()
    application.DrawAll()
    application.EndScene()