import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Create Chrono::Engine physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Visualization setup with Irrlicht
vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("MAN 10t Truck Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, -10))  # Initial camera position
vis.SetCameraChaseMode(chrono.ChVectorD(0, 1.5, 0), 8)  # Chase camera settings

# Create terrain (rigid plane)
terrain = veh.ChTerrain(system)
terrain.SetSize(100.0, 100.0)  # Size X,Z
terrain.SetHeight(0)
terrain.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)
system.Add(terrain)

# Create MAN 10t truck vehicle
vehicle = veh.MAN10ton()
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType.TMEASY)  # TMEASY tire model
vehicle.SetDriveType(veh.DrivelineTypeWV_4WD)  # 4WD driveline
vehicle.Initialize()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
system.Add(vehicle.GetChassis())

# Create driver and input controls
driver = veh.ChDriver()
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)
vehicle.SetDriver(driver)

# Simulation settings
step_size = 0.01
time = 0.0

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Handle user inputs
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
    
    # Update driver inputs
    driver.SynchronizeTime(time)
    vehicle.Synchronize(time)
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    time += step_size
    
    # Update visualization
    vis.AssetsUpdate()
    
    # Exit on ESC
    if key.KeyPressed(irr.KEYKEY_ESCAPE):
        break

# Cleanup
vis.Close()