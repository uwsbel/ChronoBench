import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("MAN 10t Truck Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, -10))  
vis.SetCameraChaseMode(chrono.ChVectorD(0, 1.5, 0), 8)  


terrain = veh.ChTerrain(system)
terrain.SetSize(100.0, 100.0)  
terrain.SetHeight(0)
terrain.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)
system.Add(terrain)


vehicle = veh.MAN10ton()
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType.TMEASY)  
vehicle.SetDriveType(veh.DrivelineTypeWV_4WD)  
vehicle.Initialize()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
system.Add(vehicle.GetChassis())


driver = veh.ChDriver()
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)
vehicle.SetDriver(driver)


step_size = 0.01
time = 0.0


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    key = vis.GetKeyboard()
    if key.KeyPressed( irr.KEYKEY_UP ):
        driver.IncreaseThrottle(0.1)
    if key.KeyPressed( irr.KEYKEY_DOWN ):
        driver.IncreaseThrottle(-0.1)
    if key.KeyPressed( irr.KEYKEY_LEFT ):
        driver.SetSteering(-1.0)
    if key.KeyPressed( irr.KEYKEY_RIGHT ):
        driver.SetSteering(1.0)
    if key.KeyReleased( irr.KEYKEY_LEFT ) or key.KeyReleased( irr.KEYKEY_RIGHT ):
        driver.SetSteering(0.0)
    
    
    driver.SynchronizeTime(time)
    vehicle.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    time += step_size
    
    
    vis.AssetsUpdate()
    
    
    if key.KeyPressed(irr.KEYKEY_ESCAPE):
        break


vis.Close()