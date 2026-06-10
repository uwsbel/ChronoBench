# =============================================================================
# PyChrono HMMWV on SCM Deformable Terrain Simulation
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# =============================================================================
# Simulation Parameters
# =============================================================================
step_size = 1e-3          # Integration step size
tire_step_size = 1e-3     # Tire step size
render_fps = 50           # Rendering frames per second
render_step_size = 1.0 / render_fps

# Terrain dimensions
terrainLength = 16.0      # Size in X direction
terrainWidth = 8.0        # Size in Y direction

# Vehicle initial position and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# =============================================================================
# Initialize PyChrono and Vehicle System
# =============================================================================
# Set data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create the HMMWV vehicle system
hmmwv = veh.HMMWV_Full()

# Vehicle system settings
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)

# Initial vehicle position and orientation
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))

# Powertrain and drivetrain settings
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)

# Set tire model to RIGID
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(tire_step_size)

# Initialize the HMMWV
hmmwv.Initialize()

# Apply MESH visualization to all vehicle components
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Retrieve the vehicle system and chassis body
vehicle_system = hmmwv.GetSystem()

# =============================================================================
# Configure SCM Deformable Terrain
# =============================================================================
terrain = veh.SCMTerrain(vehicle_system)

# Set soil parameters (Bekker-Wong model parameters)
# Bekker parameters
terrain.SetSoilParameters(
    2e6,   # Bekker Kphi    [N/m^(2+n)]
    0,     # Bekker Kc      [N/m^(1+n)]
    1.1,   # Bekker n exponent (dimensionless)
    0,     # Mohr cohesive limit [Pa]
    30,    # Mohr friction limit [degrees]
    0.01,  # Janosi shear coefficient k [m]
    2e8,   # Elastic stiffness (not used if zero)
    3e4    # Damping coefficient [Ns/m^2]
)

# Enable bulldozing (soil pushed in front of wheels)
terrain.EnableBulldozing(True)
terrain.SetBulldozingParameters(
    55,    # Angle of friction for bulldozing flow [degrees]
    1,     # Ratio of lateral to normal force
    5,     # Bulldozing flow factor
    3,     # Max number of iterations
    10     # Width of the bulldozing effects (mm)
)

# Enable moving patch that follows the vehicle chassis
terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),          # Body to follow
    chrono.ChVector3d(0, 0, 0),      # Patch center offset relative to body
    chrono.ChVector3d(5.0, 3.0, 1.0) # Patch dimensions (length, width, height)
)

# Set SCM grid resolution
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize terrain with flat surface
terrain.Initialize(
    terrainLength,    # Size in X
    terrainWidth,     # Size in Y
    0.02              # Grid mesh resolution [m]
)

# =============================================================================
# Create Interactive Driver System
# =============================================================================
driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())

# Set driver input gains
driver.SetSteeringDelta(render_step_size / 1.0)   # Steering response
driver.SetThrottleDelta(render_step_size / 8.0)   # Throttle response
driver.SetBrakingDelta(render_step_size / 2.0)    # Braking response

# =============================================================================
# Create Irrlicht Visualization
# =============================================================================
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()

# Set window title and size
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetWindowSize(1280, 720)

# Set chase camera parameters
vis.SetChaseCamera(
    chrono.ChVector3d(0.0, 0.0, 0.75),  # Camera offset from vehicle
    6.0,                                  # Camera distance
    0.5                                   # Camera damping
)

# Initialize visualization
vis.Initialize()

# Add typical lights and skybox
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_chrono_alpha.png')

# Attach visualization to vehicle
vis.AttachVehicle(hmmwv.GetVehicleInstance())

# Initialize driver with visualization
driver.Initialize()

# =============================================================================
# Simulation Loop
# =============================================================================
print("HMMWV SCM Terrain Simulation Starting...")
print(f"Step size: {step_size} s")
print(f"Render FPS: {render_fps}")
print("Use keyboard controls:")
print("  Arrow keys: Steering and throttle")
print("  Space: Braking")
print("  Q/ESC: Quit")

# Real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

# Simulation time tracking
sim_time = 0.0
render_time = 0.0
step_number = 0

while vis.Run():
    # Current simulation time
    sim_time = hmmwv.GetSystem().GetChTime()

    # Render at specified FPS
    if sim_time >= render_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_time += render_step_size

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update subsystems
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    hmmwv.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    # Advance simulation by one step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    # Increment step counter
    step_number += 1

    # Maintain real-time simulation
    realtime_timer.Spin(step_size)

print("Simulation completed.")