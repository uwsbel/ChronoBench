import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain = chrono.ChRigidBody()
terrain.SetBodyFixed(True)
terrain.SetMass(0)
terrain.SetPos(chrono.ChVectorD(0, -0.1, 0))
terrain.SetShapeType(chrono.ChShapeType.BOX)
terrain.SetBox(100, 0.1, 100)
system.Add(terrain)


try:
    texture = chronoirr.ChTexture("terrain.png")  
except:
    print("Error: Could not load terrain texture. Using default color.")
    texture = None


vehicle = veh.FEDA_Vehicle(system)


vehicle.SetVehiclePos(chrono.ChVectorD(0, 1, 0))
vehicle.SetVehicleRot(chrono.ChQuaternionD(0, 0, 0))
vehicle.SetContactMethod(veh.ChContactMethod.NSC)
vehicle.SetTireModel(veh.ChTireModel.RADIAL)
vehicle.SetEngineType(veh.ChEngineModelType.SIMPLE)


vehicle.SetVisualizationType(veh.ChVehicleVisualizationType.MESH)


driver = veh.ChInteractiveDriver(vehicle)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEDA Vehicle Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()


camera_offset = chrono.ChVectorD(0, 2, -5)
camera_up = chrono.ChVectorD(0, 1, 0)


time_step = 1.0 / 50.0  
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Synchronize(time_step)

    
    system.DoStepDynamics(time_step)

    
    camera_pos = vehicle.GetPos() + camera_offset
    vis.GetVisualSystem().GetIrrlichtDevice().camera->setPosition(camera_pos)
    vis.GetVisualSystem().GetIrrlichtDevice().camera->setUpVector(camera_up)