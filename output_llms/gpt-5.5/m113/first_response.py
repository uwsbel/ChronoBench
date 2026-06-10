# =============================================================================
# PyChrono M113 tracked vehicle demo with rigid terrain and Irrlicht visualization
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


# -----------------------------------------------------------------------------
# 1. Global Chrono / Vehicle initialization
# -----------------------------------------------------------------------------

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Simulation parameters
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50.0
render_steps = int(render_step_size / step_size)

# Initial vehicle conditions
init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.QuatFromAngleZ(0.0)
init_fwd_vel = 0.0

# Terrain parameters
terrain_length = 200.0
terrain_width = 200.0
terrain_friction = 0.9
terrain_restitution = 0.01

# Visualization parameters
track_point = chrono.ChVector3d(0.0, 0.0, 1.0)
camera_distance = 8.0
camera_height = 1.5


# -----------------------------------------------------------------------------
# 2. Create and configure the M113 vehicle
# -----------------------------------------------------------------------------

m113 = veh.M113()

m113.SetContactMethod(contact_method)
m113.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# Set optional initial forward velocity if supported by installed Chrono version
if hasattr(m113, "SetInitFwdVel"):
    m113.SetInitFwdVel(init_fwd_vel)

# Vehicle subsystem models
m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
m113.SetEngineType(veh.EngineModelType_SIMPLE)
m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
m113.SetBrakeType(veh.BrakeType_SIMPLE)

# Initialize vehicle
m113.Initialize()

# Visualization types
m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)

# Access the underlying Chrono system
system = m113.GetSystem()

# Solver settings
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(150)

# Enable real-time mode for the vehicle
m113.GetVehicle().EnableRealtime(True)


# -----------------------------------------------------------------------------
# 3. Create rigid terrain
# -----------------------------------------------------------------------------

terrain = veh.RigidTerrain(system)

terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(terrain_friction)
terrain_mat.SetRestitution(terrain_restitution)

terrain_patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0.0, 0.0, 0.0),
        chrono.QUNIT
    ),
    terrain_length,
    terrain_width
)

terrain_patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    terrain_length / 2.0,
    terrain_width / 2.0
)

terrain.Initialize()


# -----------------------------------------------------------------------------
# 4. Create Irrlicht visualization system
# -----------------------------------------------------------------------------

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(track_point, camera_distance, camera_height)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(m113.GetVehicle())


# -----------------------------------------------------------------------------
# 5. Create interactive driver system
# -----------------------------------------------------------------------------

driver = veh.ChInteractiveDriverIRR(vis)

# Time required to go from 0 to full input
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


# -----------------------------------------------------------------------------
# 6. Simulation loop
# -----------------------------------------------------------------------------

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = system.GetChTime()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    m113.Advance(step_size)
    vis.Advance(step_size)

    # Maintain real-time execution
    realtime_timer.Spin(step_size)

    step_number += 1