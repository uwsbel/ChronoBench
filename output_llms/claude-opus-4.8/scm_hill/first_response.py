import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# -----------------------------------------------------------------------------
# 1. Initialize PyChrono environment and core components
# -----------------------------------------------------------------------------

# Set the data path for vehicle and terrain assets
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-5, -2, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Simulation step size (small for deformable terrain stability)
step_size = 2e-3
tire_step_size = 1e-3

# -----------------------------------------------------------------------------
# 2. Create and configure the HMMWV vehicle
# -----------------------------------------------------------------------------

# Create the HMMWV vehicle, set parameters, and initialize
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)

# Set the initial position and orientation
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))

# Drivetrain and tire model selection
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(tire_step_size)

# Initialize the vehicle
hmmwv.Initialize()

# Set visualization types for vehicle subsystems
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Get the underlying Chrono system
system = hmmwv.GetSystem()

# -----------------------------------------------------------------------------
# 3. Create the SCM deformable terrain
# -----------------------------------------------------------------------------

# Terrain dimensions and mesh resolution
terrain_length = 20.0   # X direction (m)
terrain_width = 20.0    # Y direction (m)
delta = 0.05            # Mesh resolution (m)

# Create the SCM terrain
terrain = veh.SCMTerrain(system)

# Set the plane reference (orientation of terrain)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                                    chrono.QuatFromAngleX(-math.pi / 2)))

# Initialize the terrain from a height map image (heightmap-based bumps)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/test64.bmp"),  # height map image
    terrain_length,   # sizeX
    terrain_width,    # sizeY
    0.0,              # min height
    1.0,              # max height
    delta             # mesh resolution
)

# Set the soil (Bekker-Wong) parameters for the deformable terrain
terrain.SetSoilParameters(
    2e6,    # Bekker Kphi
    0,      # Bekker Kc
    1.1,    # Bekker n exponent
    0,      # Mohr cohesive limit (Pa)
    30,     # Mohr friction limit (degrees)
    0.01,   # Janosi shear coefficient (m)
    2e8,    # Elastic stiffness (Pa/m)
    3e4     # Damping (Pa s/m)
)

# Enable bulldozing effects (optional, for more realistic terrain deformation)
terrain.EnableBulldozing(True)
terrain.SetBulldozingParameters(
    55,    # angle of friction for erosion (degrees)
    1,     # flow factor
    5,     # number of erosion refinements per timestep
    6      # number of concentric vertex selections
)

# Set automatic refinement of the mesh under the tires
terrain.SetAutomaticRefinement(True)
terrain.SetAutomaticRefinementResolution(0.04)

# Set the terrain plot type for visualization (color by sinkage)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Set mesh wireframe visualization
terrain.GetMesh().SetWireframe(True)

# -----------------------------------------------------------------------------
# 4. Initialize the driver system for vehicle control
# -----------------------------------------------------------------------------

driver = veh.ChInteractiveDriverIRR  # placeholder; created with vis below

# -----------------------------------------------------------------------------
# 5. Create the Irrlicht visualization
# -----------------------------------------------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# Create the interactive driver and attach it to the visualization
driver = veh.ChInteractiveDriverIRR(vis)

# Set time response for steering, throttle, and braking inputs
driver.SetSteeringDelta(0.04)   # rad/s
driver.SetThrottleDelta(0.04)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# -----------------------------------------------------------------------------
# 6. Simulation loop with real-time synchronization
# -----------------------------------------------------------------------------

# Real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

print("Starting simulation...")

while vis.Run():
    time = system.GetChTime()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Collect driver inputs at the current time
    driver_inputs = driver.GetInputs()

    # --- Synchronize all subsystems for the current timestep ---
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # --- Advance the state of all subsystems by one timestep ---
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    # Maintain real-time execution
    realtime_timer.Spin(step_size)

print("Simulation finished.")