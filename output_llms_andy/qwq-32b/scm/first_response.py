import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.postprocess as post

# Initialize simulation settings
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
out_dir = "./HMMWV_SCM_Simulation/"

# Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.SetCameraPosition(chrono.ChVectorD(5, 2, 5))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
vis.SetWindowSize(1280, 1024)
vis.SetAntialiasing(True)
system.SetVisualSystem(vis)

# Create HMMWV vehicle
hmmwv = veh.HMMWV_Reduced()
hmmwv.Initialize(system, True, "RIGID")  # Rigid tire model
hmmwv.SetInitPosition(chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 1.2), chrono.Q_from_Euler_123(chrono.ChVectorD(0, 0, 0))))
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)

# Configure SCM terrain
soil = veh.soil.ChSoilModel()
soil.SetSize(50, 50, 0.5)  # Terrain size (x,y) and depth
soil.SetPosition(chrono.ChVectorD(0, 0, 0))
soil.SetShearModulus(1e6)  # Custom soil parameters
soil.SetBulkModulus(1.5e6)
soil.SetDensity(1500)
soil.SetMaxStrainRate(100)
soil.SetCohesion(2e4)
soil.SetFrictionAngle(chrono.CH_C_PI / 6)
soil.SetAdhesion(50)
soil.SetUseMovingPatch(True)
soil.SetMovingPatchRadius(2.5)  # Adjust based on vehicle size

# Configure moving patch linked to chassis
moving_patch = soil.GetMovingPatch()
moving_patch.SetPosition(hmmwv.GetChassis().GetPos())
moving_patch.SetUpdatePosition(True)

# Enable false color visualization for sinkage
soil.SetVisualizationType(veh.soil.VisualizationType_SINKAGE)
soil.SetSinkageVisualization(true)

system.Add(soil)

# Driver setup
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)

# Simulation loop parameters
time_step = 0.02  # 50Hz
real_time_factor = 1.0

# Create Irrlicht driver
app = irr.ChIrrApp(system, "HMMWV on SCM Terrain", irr.dimension2du(1280, 1024))
app.SetSystem(system)
app.SetCamera(chronoirr.ChIrrCameraMetric())
app.SetSymbolscheme(chronoirr.ISS_VS_2010)
app.AssetBind()
app.AssetUpdate()

# Simulation loop
while app.GetDevice().run():
    time = system.GetChTime()
    
    # Update moving patch position
    moving_patch.SetPosition(hmmwv.GetChassis().GetPos())
    
    # Driver inputs
    driver.Synchronize(time)
    driver.DriverInputs()
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Update visualization
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    soil.Draw(app.GetVideoDriver())
    app.EndScene()
    
    # Check exit condition
    if driver.GetExit():
        break

# Cleanup
app.GetDevice().closeDevice()