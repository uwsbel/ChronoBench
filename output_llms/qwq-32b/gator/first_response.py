import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Create the Chrono system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create Irrlicht visualizer
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

# Create Gator vehicle
vehicle = veh.Gator(sys)
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
vehicle.SetTireModel(veh.TireModelType.TMEASY)

# Initialize vehicle position and orientation
init_pos = chrono.ChVectorD(0, 0, 1)  # Above terrain
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle.Initialize(chrono.ChCoordsysD(init_pos, init_rot))

# Create rigid terrain
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
terrain.SetClamping(True)
terrain.SetPatch(veh.RigidTerrain.PatchData(100, 100, 1, patch_mat))
terrain.Initialize()

# Add terrain texture
texture_path = 'textures/concrete.jpg'  # Ensure the texture file exists
terrain.GetPlane().AddAsset(chrono.ChTexture(texture_path))

# Add systems to the Chrono system
sys.Add(vehicle.GetSystem())
sys.Add(terrain.GetSystem())

# Create driver
driver = veh.UserDriver()
vehicle.SetDriver(driver)

# Simulation parameters
time_step = 0.02  # 50 Hz
time = 0

# Main simulation loop
while visualiz.Run():
    key = visualiz.GetKeyboard()
    
    # Update driver inputs based on keyboard
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
    
    driver.SetSteering(steering * 0.5)  # Scale steering input
    driver.SetThrottle(throttle)
    driver.SetBraking(brake)
    
    # Update vehicle and system
    vehicle.Synchronize(driver, time)
    vehicle.Update(time)
    sys.DoStepDynamics(time_step)
    
    # Render
    visualiz.BeginScene()
    visualiz.DrawAll()
    visualiz.EndScene()
    
    time += time_step
    
    # Exit on ESC key
    if key.KeyIsPressed(chronoirr.EDK_ESCAPE):
        break

# Cleanup
visualiz.Close()