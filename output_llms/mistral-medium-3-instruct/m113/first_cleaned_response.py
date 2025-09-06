import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.wheeled_vehicle as wheeled_vehicle
import pychrono.vehicle.terrain as terrain






sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(10, 5, 0))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.CameraVerticalDir::Z)
vis.SetCameraAngle(chrono.ChVectorD(0.5 * chrono.CH_PI, 0, 0))
vis.SetCameraTracking(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))






terrain = veh.RigidTerrain(sys)
terrain.SetContactFriction(0.8)  
terrain.SetContactRestitution(0.1)  
terrain.SetContactMaterialProperties(2e7, 0.3)  


patch_size = 100
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(patch_size, 0, 0),
                         chrono.ChVectorD(0, 0, patch_size), "flat_patch")
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)






vehicle = veh.WheeledVehicle(sys, veh.VehicleSide::DOUBLE_SIDE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetInitFwdVel(10)  


m113 = veh.M113()
m113.Initialize(vehicle)






driver = veh.ChDriver()
driver.Initialize(vehicle)


path = veh.ChPath()
path.AddPoint(0, 0, 0)
path.AddPoint(50, 0, 0)
path.AddPoint(100, 0, 0)
driver.SetPath(path)


driver.SetSteeringControllerType(veh.ChSteeringController::PATH_FOLLOWING)
driver.SetThrottleControllerType(veh.ChThrottleController::SPEED)
driver.SetDesiredSpeed(10)  






vehicle.Initialize()


veh.SetChassisVisualizationType(vehicle, veh.VisualizationType::MESH)
veh.SetSuspensionVisualizationType(vehicle, veh.VisualizationType::PRIMITIVES)
veh.SetSteeringVisualizationType(vehicle, veh.VisualizationType::PRIMITIVES)
veh.SetWheelVisualizationType(vehicle, veh.VisualizationType::MESH)






time_step = 0.01  
sim_time = 10.0  


while vis.Run():
    time = sys.GetChTime()

    
    if time > sim_time:
        break

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, veh.WheeledVehicle::DriverInputs(driver.GetSteering(), driver.GetThrottle(), driver.GetBraking()))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    
    if int(time * 10) % 10 == 0:
        pos = vehicle.GetChassis().GetPos()
        vel = vehicle.GetChassis().GetPos_dt()
        print(f"Time: {time:.2f} s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) m | Speed: {vel.Length():.2f} m/s")