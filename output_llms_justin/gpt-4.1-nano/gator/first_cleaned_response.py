import pychrono.core as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('../data/')  
system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, 'Gator Vehicle Simulation', irr.dimension2du(1024, 768))
application.GetDevice().setWindowCaption('PyChrono Gator Vehicle on Flat Terrain')
application.DrawAllDeletedInsteadOfHidden(False)


application.GetVideoDriver().setAutoClearColor(irr.SColor(255, 140, 190, 230))
application.GetSceneManager().addCameraSceneNode(irr.ux().getCamera())


terrain = chrono.ChRigidTerrain(system)
terrain.Reset()
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
terrain.GetGround().SetTextureFilename('textures/concrete.jpg')
terrain.GetGround().SetTextureLength(10)

terrain.Initialize(chrono.VNULL, 200, 200, 0.3)



gator = chrono.ChGATOR()

initial_pos = chrono.ChVectorD(0, 0, 0.5)  
initial_rot = chrono.Q_from_AngY(0)  
gator.SetPos(initial_pos)
gator.SetRot(initial_rot)


gator.SetContactMethod(chrono.ChContactMethod_Chaos)
gator.GetChassis().SetMeshName('models/gator_chassis.obj')  
gator.GetWheelAssembly(0).GetTire().SetTireModel(chrono.VehicleTireModel_TMEASY)



gator.SetMeshVisualizationType(chrono.VehicleMeshVisualizationType_Mesh)


system.Add(gator.GetVehicle())


gator.Initialize(initial_pos, initial_rot)


driver = chrono.ChIrrGuiDriver(application)
driver.SetThrottleDelta(0.01)
driver.SetSteeringDelta(0.5)
driver.SetBrakingDelta(0.02)


fps = 50
application.SetTimeStep(1.0 / fps)


while application.GetDevice().run():
    
    application.BeginScene()
    application.DrawAll()

    
    driver.ProcessInputs()
    steering_input = driver.GetSteering()
    throttle_input = driver.GetThrottle()
    brake_input = driver.GetBrake()

    
    gator.GetSteeringLink()->SetSteeringAngle(steering_input)
    gator.GetMotor()->SetAppliedTorque(throttle_input * gator.GetMaxMotorTorque())
    gator.GetBraking()->Set_braking(brake_input)

    
    system.DoStepDynamics(1.0 / fps)

    
    application.EndScene()