import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os




chrono.SetChronoDataPath(os.getcwd())


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = chronoirr.ChIrrApp(system, 'FEDA Vehicle Simulation', chronoirr.dimension2du(1280, 720))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_splash.png'))
application.AddTypicalUnitSystems()
application.AddTypicalCamera(chrono.ChVectorD(0, 2, 10), chrono.ChVectorD(0, 2, 0))


vehicle = chrono.vehicle.FEDAVehicle()
vehicle.SetChassisBodyVisualizationType(chrono.vehicle.ChVisualizationType_MESH)
vehicle.SetChassisBodyMeshFilename(chrono.GetChronoDataFile('vehicles/feda/body.obj'))
vehicle.SetInitPosition(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetInitOrientation(chrono.Q_from_AngAxis(chrono.ChMatrix33D(1)))
vehicle.SetTireType(chrono.vehicle.FEDAVehicle.TIRE_PACEJKA)
vehicle.SetTireContactMethod(chrono.vehicle.FEDAVehicle.TIRE_CONTACT_SINGLE_CONTACT_POINT)


system.Add(vehicle.GetVehicleSystem())


terrain = chrono.ChTerrain()
terrain.SetTextureFilename(chrono.GetChronoDataFile('textures/concrete.jpg'))
terrain.SetPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
terrain.SetSize(chrono.ChVectorD(100, 0.1, 100))
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(terrain)


driver = chrono.vehicle.ChDriver()
driver.SetVehicle(vehicle)
driver.SetSteeringController(chrono.vehicle.ChGeneric_2DofSteeringController())
driver.SetThrottleController(chrono.vehicle.ChGeneric_PIDThrottleController())
driver.SetBrakingController(chrono.vehicle.ChGeneric_PIDBrakingController())


camera = application.GetSceneManager().getActiveCamera()
camera.setPosition(chrono.ChVectorD(0, 2, 10))
camera.setTarget(chrono.ChVectorD(0, 2, 0))


application.SetTimestep(1.0 / 50)
application.SetTryRealTimeRate(1.0 / 50)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()


application.GetDevice().closeDevice()