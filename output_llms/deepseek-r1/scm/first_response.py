import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# 1. Initialize PyChrono environment
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
terrain.Initialize(100.0, 60.0, 0.04)  # 100x60 meter terrain with 4cm resolution

# Configure soil parameters (moist clay example)
soil_params = veh.SoilParametersCallback()
soil_params.m_Becker = 134.3e3
soil_params.m_Kappa = 1.0e8
soil_params.m_R = 5000
soil_params.m_phi = 31.1
soil_params.m_Kcoh = 0.2e3
soil_params.m_n = 1.1
terrain.SetSoilParameters(soil_params)

# Enable moving patch that follows the vehicle
terrain.AddMovingPatch(veh.ChVectorD(0, 0, 0), 5.0, 3.0)  # Will be updated later

# Set visualization properties
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.15)
terrain.GetMesh().SetWireframe(True)

# 3. Create HMMWV vehicle
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 0.5),  # Start 0.5m above terrain
    chrono.Q_from_AngZ(0)  # Initial orientation
))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.Initialize()

# Set visualization modes
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# 4. Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(8, 8, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()

# 5. Create interactive driver
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# 6. Configure simulation parameters
step_size = 0.002
render_step = 1.0 / 50  # 50 FPS
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(50)

# Main simulation loop
time = 0
realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = system.GetChTime()
    
    # Update moving patch to follow vehicle chassis
    chassis_pos = hmmwv.GetVehicle().GetChassisBody().GetPos()
    terrain.SetMovingPatchCenter(chassis_pos)
    
    # Render scene
    if time % render_step < step_size:
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