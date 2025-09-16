import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


visualiz = irr.ChVisualSystemIrrlicht()
visualiz.SetWindowSize(1024, 768)
visualiz.SetWindowTitle('Gator Simulation')
visualiz.Initialize()
visualiz.AddLogo()
visualiz.AddSky()
visualiz.AddGrid()
visualiz.SetCameraPosition(chrono.ChVectorD(0, 2, 10))
visualiz.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0)))
sys.SetVisualSystem(visualiz)


vehicle = veh.Gator(sys)
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
vehicle.SetTireModel(veh.TireModelType.TMEASY)


init_pos = chrono.ChVectorD(0, 0, 1)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle.Initialize(chrono.ChCoordsysD(init_pos, init_rot))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
terrain.SetClamping(True)
terrain.SetPatch(veh.RigidTerrain.PatchData(100, 100, 1, patch_mat))
terrain.Initialize()


texture_path = 'textures/concrete.jpg'  
terrain.GetPlane().AddAsset(chrono.ChTexture(texture_path))


sys.Add(vehicle.GetSystem())
sys.Add(terrain.GetSystem())


driver = veh.UserDriver()
vehicle.SetDriver(driver)


time_step = 0.02  
time = 0


while visualiz.Run():
    key = visualiz.GetKeyboard()
    
    
    steering = 0
    throttle = 0
    brake = 0
    
    if key.KeyIsPressed(chronoirr.EDK_LEFT):
        steering -= 1
    if key.KeyIsPressed(chronoirr.EDK_RIGHT):
        steering += 1
    if key.KeyIsPressed(chronoirr.EDK_UP):
        throttle = 1
    if key.KeyIsPressed(chronoirr.EDK_DOWN):
        brake = 1
    
    driver.SetSteering(steering * 0.5)  
    driver.SetThrottle(throttle)
    driver.SetBraking(brake)
    
    
    vehicle.Synchronize(driver, time)
    vehicle.Update(time)
    sys.DoStepDynamics(time_step)
    
    
    visualiz.BeginScene()
    visualiz.DrawAll()
    visualiz.EndScene()
    
    time += time_step
    
    
    if key.KeyIsPressed(chronoirr.EDK_ESCAPE):
        break


visualiz.Close()