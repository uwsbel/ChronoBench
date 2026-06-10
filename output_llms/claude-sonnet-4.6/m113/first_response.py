import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# =============================================================================
# Initialize PyChrono environment
# =============================================================================

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# =============================================================================
# M113 Vehicle Initialization
# =============================================================================

# Create the M113 vehicle
m113 = veh.M113()

# Set initial vehicle position and orientation
init_loc = chrono.ChVector3d(0, 0, 1.1)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation (facing +X direction)

m113.SetContactMethod(chrono.ChContactMethod_SMC)
m113.SetChassisFixed(False)
m113.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
m113.SetBrakeType(veh.BrakeType_SIMPLE)
m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
m113.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
m113.SetTransmissionType(veh.TransmissionModelType_SIMPLE_MAP)

# Initialize the M113 vehicle
m113.Initialize()

# Set visualization types
m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)

# =============================================================================
# Terrain Setup
# =============================================================================

# Create the terrain
terrain = veh.RigidTerrain(m113.GetSystem())

# Define terrain patch properties
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)          # Friction coefficient
patch_mat.SetRestitution(0.01)      # Restitution coefficient
patch_mat.SetYoungModulus(2e7)      # Young's modulus

# Add a flat patch
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    200.0,  # Terrain length (X direction)
    200.0   # Terrain width (Y direction)
)

# Set terrain visualization
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Initialize the terrain
terrain.Initialize()

# =============================================================================
# Driver System
# =============================================================================

# Create an interactive driver system
driver = veh.ChInteractiveDriverIRR(vis := irr.ChVisualSystemIrrlicht())

# Set up driver input time responses
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

# =============================================================================
# Irrlicht Visualization Setup
# =============================================================================

# Create the Irrlicht visualization system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(m113.GetSystem())
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.SetWindowSize(1280, 720)
vis.Initialize()

# Add logo and sky box
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Add lighting
vis.AddLightDirectional()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),  # Light position
    chrono.ChVector3d(0, 0, 0),           # Target point
    3,                                     # Radius
    2.2, 7.2,                             # Near/far clip
    40,                                   # Angle of FOV
    512                                   # Shadow map resolution
)

# Set up chase camera
vis.SetChaseCamera(
    chrono.ChVector3d(0.0, 0.0, 0.0),  # Camera target point offset
    6.0,                                 # Chase distance
    0.5                                  # Chase height
)
vis.SetChaseCameraState(irr.utils.ChChaseCamera.Chase)
vis.SetChaseCameraPosition(m113.GetChassisBody().GetPos() + chrono.ChVector3d(-6, 0, 2))

# =============================================================================
# Recreate driver with visualization system properly initialized
# =============================================================================

# Reinitialize driver with the visualization system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# =============================================================================
# Simulation Loop Parameters
# =============================================================================

# Simulation step size
step_size = 5e-4

# Render frame rate
render_step_size = 1.0 / 50  # 50 FPS
render_steps = math.ceil(render_step_size / step_size)

# Real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

# =============================================================================
# Simulation Loop
# =============================================================================

print("Starting M113 simulation...")
print("Use arrow keys to control the vehicle:")
print("  Up/Down    - Throttle/Brake")
print("  Left/Right - Steering")

step_number = 0
render_frame = 0

while vis.Run():
    # Get current simulation time
    time = m113.GetSystem().GetChTime()

    # Render scene at specified frame rate
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize systems at current time
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    m113.Advance(step_size)
    vis.Advance(step_size)

    # Maintain real-time execution
    realtime_timer.Spin(step_size)

    # Increment step counter
    step_number += 1

print("Simulation completed.")