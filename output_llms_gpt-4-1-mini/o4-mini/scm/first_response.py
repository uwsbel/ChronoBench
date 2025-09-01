import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# --------------------------------------------------------------------------
# 1) Initialize Chrono and set the data path
# --------------------------------------------------------------------------
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # e.g. '/usr/local/share/chrono/'
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.002)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)

# --------------------------------------------------------------------------
# 2) Simulation parameters
# --------------------------------------------------------------------------
time_step = 1e-3         # integration time step
render_fps = 50          # desired rendering frame rate
render_frame_step = int(1.0 / render_fps / time_step)

# --------------------------------------------------------------------------
# 3) Create the vehicle (full HMMWV)
# --------------------------------------------------------------------------
# Instantiate the HMMWV vehicle with default file locations
vehicle = veh.HMMWV_Full()
# Set initial position and orientation
init_loc = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.Q_from_AngY(0.0)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetTireType(veh.VehicleTireType.RIGID)
# Set mesh visualization for all subsystems
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
# Initialize the vehicle system
vehicle.Initialize()

# --------------------------------------------------------------------------
# 4) Create the SCM deformable terrain
# --------------------------------------------------------------------------
terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
# Specify a plane for the terrain
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
# Set custom soil parameters: [Kphi, Kc, n, cohesion, friction, density, damping]
terrain.SetSoilParameters(
    2e6,     # Bekker Kphi  
    0.0,     # Bekker Kc
    1.1,     # Bekker n
    0.0,     # cohesion
    30.0,    # friction angle (deg)
    1500.0,  # density (kg/m3)
    3e4      # damping
)
# Enable bulldozing effects (optional)
terrain.SetBulldozingFlow(True)
terrain.SetBulldozingParameters(0.1, 1.0, 2.0)
# Enable a moving patch that follows the chassis
patch_length = 6.0
patch_width  = 6.0
terrain.EnableMovingPatch(vehicle.GetChassisBody(), patch_length, patch_width)
# Visualize sinkage with false–color plot
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE,
                    veh.SCMDeformableTerrain.COLOR_MAP)
# Initialize the terrain mesh (large enough so that initial position is covered)
terrain.Initialize()

# --------------------------------------------------------------------------
# 5) Create the Irrlicht visualization application
# --------------------------------------------------------------------------
app = irr.ChIrrApp(
    vehicle.GetSystem(),                                    # the Chrono::ChSystem
    "HMMWV on SCM Terrain",                                 # window title
    irr.dimension2du(1280, 720),                            # window dims
    irr.VerticalDir_Z                                        # z-up
)
app.AddLogo()
app.AddSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(trackPoint=chrono.ChVectorD(0, 0, 1.0),
                   dist=6.0,
                   rot=chrono.Q_from_AngAxis(chrono.CH_C_PI / 6, chrono.ChVectorD(1, 0, 0)))
# Bind and update all assets before simulation
app.AssetBindAll()
app.AssetUpdateAll()

# --------------------------------------------------------------------------
# 6) Create the interactive driver
# --------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.05)
driver.Initialize()

# Tell the Irrlicht application to use the same step size
app.SetTimestep(time_step)
app.SetTryRealtime(True)

# --------------------------------------------------------------------------
# 7) Simulation loop
# --------------------------------------------------------------------------
step_number = 0
while app.GetDevice().run():
    # Begin scene
    app.BeginScene()
    app.DrawAll()

    # Get driver inputs
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking  = driver.GetBraking()

    # Synchronize modules
    time = vehicle.GetSystem().GetChTime()
    terrain.Synchronize(time)
    vehicle.Synchronize(time, steering, throttle, braking, terrain)

    # Render and end scene
    app.EndScene()

    # Advance simulation for one time step
    vehicle.Advance(time_step)
    terrain.Advance(time_step)

    step_number += 1

# end while