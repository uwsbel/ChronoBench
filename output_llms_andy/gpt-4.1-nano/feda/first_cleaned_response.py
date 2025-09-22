import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('')  


sys = chrono.ChSystemNSC()


application = irr.ChIrrApp(sys, "FEDA Vehicle on Rigid Terrain", irr.dimension2d(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 3, -10))
application.AddTypicalLights()
application.SetTimestep(1.0 / 50.0)  


terrain = veh.ChRigidTerrain(sys)
terrain.SetContactMethod(veh.ChContactMethod_NSC)
terrain.Initialize(veh.ChVector(0, 0, 0))

terrain.GetGround()->GetMaterialSurface()->SetTexture(veh.GetChronoDataFile("textures/concrete.jpg"))
terrain.GetGround()->GetMaterialSurface()->SetFriction(0.9)



initial_pos = chrono.ChVector(0, 0.5, 0)
initial_rot = chrono.ChQuaternion(1, 0, 0, 0)  


vehicle = veh.FEDA_Vehicle(sys, veh.ChassisCollisionType_MESH, True)
vehicle.SetChassisPosition(initial_pos)
vehicle.SetChassisOrientation(initial_rot)
vehicle.SetContactMethod(veh.ChContactMethod_NSC)
vehicle.Initialize()


for part in vehicle.GetVehicleModel().GetAllParts():
    part.SetVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(application)
driver.Initialize()


application.GetSceneManager().AddCamera(irr.ChCameraLinkedToVehicle(vehicle.GetChassis(), 2.5, 1.75, 0.0))
application.SetContactsDrawMode(irr.ChIrrTools.kDrawContacts)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    driver.GetInputDriver().SetThrottle(driver.GetInputDriver().GetThrottle() + driver.GetInputDriver().GetThrottleInput() * 0.01)
    driver.GetInputDriver().SetSteering(driver.GetInputDriver().GetSteering() + driver.GetInputDriver().GetSteeringInput() * 0.01)
    driver.GetInputDriver().SetBraking(driver.GetInputDriver().GetBraking() + driver.GetInputDriver().GetBrakingInput() * 0.01)

    
    vehicle.GetDriveTrain()->SetThrottle(driver.GetInputDriver().GetThrottle())
    vehicle.GetSteering()->Set(driver.GetInputDriver().GetSteering())
    vehicle.GetBrake()->Set(driver.GetInputDriver().GetBraking())

    
    sys.DoStepDynamics(1.0 / 50.0)

    application.EndScene()