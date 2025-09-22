import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math

# -----------------------------------------------------------------------------
# Set Chrono data directory
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
# Simulation step size
step_size = 0.005  # seconds

# Simulation end time
t_end = 20.0      # seconds

# Time interval for tire output (not used in this basic example)
# output_step_size = 0.1

# Steering change parameters
steering_change_duration = 10.0 # seconds over which steering changes
max_steering_input = 0.5      # radians (approx 28 degrees)
target_speed = 1.0            # m/s

# Initial rover position and orientation
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0) # No rotation

# Camera parameters
camera_chase_dist = 6.0
camera_height = 1.5
camera_angle = 0.2 # radians, slight downward look

# -----------------------------------------------------------------------------
# Create the Chrono system
# -----------------------------------------------------------------------------
print("Creating Chrono system...")
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Set solver settings
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # Or Type_APGD, Type_SOR
sys.GetSolver().AsIterative().SetMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)

# -----------------------------------------------------------------------------
# Create the rigid terrain
# -----------------------------------------------------------------------------
print("Creating rigid terrain...")
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain_patch = terrain.AddPatch(patch_mat,
                                 chrono.CSYSNORM, # Centered at origin, Z up
                                 200.0, 200.0)    # Length and width

terrain_patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5)) # Greenish
terrain_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()
print("Terrain initialized.")

# -----------------------------------------------------------------------------
# Create the Viper rover
# -----------------------------------------------------------------------------
print("Creating Viper rover...")
viper = veh.Viper(sys)

viper.SetChassisFixed(False) # Not fixed
viper.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# --- Driveline:
# For Viper, driveline is part of the rover class, usually simple (e.g. SimpleDriveline)
# or specific configurations. Default Viper setup handles this internally.

# --- Tires:
# Viper typically uses its own specific tire models (e.g., ViperCosseratTire, ViperTMeasyTire)
# Default is usually ViperTMeasyTire.
viper.SetTireType(veh.TireModelType_TMEASY) # Options: RIGID, FIALA, TMEASY, PAC89, PAC02, COSSERAT
# If using specific Viper tires:
# viper.SetTireType(veh.TireModelType_RIGID_MESH) # if meshes are desired for rigid tires
# viper.SetTireType(veh.TireModelType_TMEASY) # Or specific Viper tire models if available and needed

viper.SetTireCollisionType(veh.Chassis.CollisionType_PRIMITIVES) # PRIMITIVES, MESH, NONE

# --- Initialize Viper:
viper.Initialize()
print("Viper rover initialized.")

# Set visualization types for rover components
viper.SetChassisVisualizationType(veh.VisualizationType_MESH)
viper.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetWheelVisualizationType(veh.VisualizationType_MESH)
viper.SetTireVisualizationType(veh.VisualizationType_MESH) # Or PRIMITIVES

# -----------------------------------------------------------------------------
# Create the Viper driver system
# -----------------------------------------------------------------------------
print("Creating Viper driver...")
# Using a simple speed controller driver
driver = veh.ViperSpeedDriver(viper.GetVehicle(), target_speed)
driver.Initialize()
print("Driver initialized.")

# -----------------------------------------------------------------------------
# Create the Irrlicht visualization system
# -----------------------------------------------------------------------------
print("Creating Irrlicht visualization...")
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowTitle('Viper Rover on Rigid Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(viper.GetChassisBody().GetFrameCOMToAbs().GetPos(), camera_chase_dist, camera_height)
vis.SetChaseCameraAngle(camera_angle) # Set fixed angle relative to rover
vis.SetChaseCameraState(chronoirr.ChChaseCamera.State_ChaseRigid) # Keep camera orientation fixed relative to rover
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddTypicalLights()

# Bind all visual assets
vis.BindAll()
print("Visualization initialized.")

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
print(f"Starting simulation loop for {t_end} seconds...")
time = 0.0
frame_number = 0

while vis.Run() and time < t_end:
    current_time = sys.GetChTime()

    # --- Update driver inputs (steering, throttle, braking) ---
    # Calculate current steering input based on time
    if current_time < steering_change_duration:
        steering_input = (current_time / steering_change_duration) * max_steering_input
    else:
        steering_input = max_steering_input
    # Could also make it oscillate, e.g., steering_input = max_steering_input * math.sin(current_time * 0.5)

    driver.SetSteering(steering_input)
    driver.SetThrottle(1.0) # Full throttle towards target_speed
    driver.SetBraking(0.0)

    # --- Synchronize systems ---
    driver.Synchronize(current_time)
    terrain.Synchronize(current_time)
    viper.Synchronize(current_time, driver.GetSteering(), driver.GetDrivingMode(), driver.GetBraking(), terrain)
    # For Viper, GetDrivingMode() often maps to forward/reverse and GetBraking() for braking.
    # The speed controller handles throttle internally to achieve target speed.

    # --- Advance simulation ---
    sys.DoStepDynamics(step_size)

    # --- Update visualization ---
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # --- Print simulation time (optional) ---
    if frame_number % 100 == 0: # Print every 100 steps
        print(f"Time: {current_time:.2f} s, Steering: {steering_input:.3f} rad")
        # print(f"  Rover Pos: {viper.GetChassisBody().GetPos()}")
        # print(f"  Rover Speed: {viper.GetChassisBody().GetPosDt().Length()}")

    time += step_size
    frame_number += 1

print("Simulation finished.")
# Note: vis.Run() will exit if the window is closed.
# No explicit vis.Quit() is strictly necessary if the loop condition handles it.