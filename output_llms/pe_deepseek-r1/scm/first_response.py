import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# 1. Initialize Chrono system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))

# 2. Create HMMWV vehicle
hmmwv = veh.HMMWV_Vehicle("HMMWV", veh.RigidTerrain.VehicleTerrain.SCM)
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 1.0),  # Initial position (z=1m above terrain)
    chrono.ChQuaterniond(1, 0, 0, 0)  # Orientation
))
hmmwv.SetTireType(veh.TireModelType_RIGID)  # Rigid tire model
hmmwv.Initialize()

# Enable mesh visualization for all vehicle components
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# 3. Configure SCM deformable terrain
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    Bekker_Kphi=0.82e6,    # [Pa]
    Bekker_Kc=0.14e4,      # [Pa]
    Bekker_n=1.0,
    Mohr_cohesion=0.017e4, # [Pa]
    Mohr_friction=30,      # [degrees]
    Janosi_shear=0.01e-2   # [m]
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.2)  # False color sinkage plot
terrain.Initialize(20, 40, 0.04)  # Dimensions: length=20m, width=40m, resolution=4cm

# Enable moving patch that follows the vehicle
terrain.AddMovingPatch(hmmwv.GetChassisBody(), chrono.ChVector3d(0, 0, 0), 5.0, 3.0)

# 4. Create interactive driver
driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

# 5. Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 8, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# Attach vehicle systems to visualization
hmmwv.GetVehicle().SetVisualSystem(vis)
driver.SetVisualSystem(vis)

# 6. Simulation loop
step_size = 0.002
frame_interval = 0.02  # 50 FPS
time = 0
step = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update vehicle systems
    driver.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Enforce real-time simulation
    realtime_timer.Spin(step_size)
    
    # Output frame at 50 FPS
    if step % int(frame_interval / step_size) == 0:
        vis.WriteImageToFile(f"frame_{step:05d}.png")
    
    step += 1