import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# ------------------------------------------------------------------------------
# Set the path to the Chrono data files
# ------------------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

# ------------------------------------------------------------------------------
# Script parameters
# ------------------------------------------------------------------------------

# Simulation step size
step_size = 0.005  # Corresponds to 200 Hz, good for SMC
fps = 50           # Frames per second for Irrlicht
render_step_size = 1.0 / fps # Timestep for rendering

# Initial vehicle position and orientation
init_loc = chrono.ChVectorD(0, 0.5, 0)  # x, y, z - y should be slightly above terrain
init_rot = chrono.ChQuaternionD(1, 0, 0, 0) # No initial rotation

# Contact method
contact_method = chrono.ChContactMethod_SMC # Use Smooth-Minima Contact (SMC)

# Tire model
tire_model = veh.TireModelType_TMEASY

# Visualization type for vehicle components
vis_type_chassis = veh.VisualizationType_MESH
vis_type_suspension = veh.VisualizationType_MESH
vis_type_steering = veh.VisualizationType_MESH
vis_type_wheel = veh.VisualizationType_MESH
vis_type_tire = veh.VisualizationType_MESH # TMEASY can also use MESH

# Paths to custom terrain mesh files
# Assumes 'data/meshes/' subdirectory relative to this script
script_dir = os.path.dirname(__file__)
terrain_mesh_coll = os.path.join(script_dir, "data", "meshes", "Highway_col.obj")
terrain_mesh_vis = os.path.join(script_dir, "data", "meshes", "Highway_vis.obj")

# Check if terrain files exist
if not os.path.exists(terrain_mesh_coll):
    print(f"Error: Collision mesh not found at {terrain_mesh_coll}")
    exit()
if not os.path.exists(terrain_mesh_vis):
    print(f"Error: Visualization mesh not found at {terrain_mesh_vis}")
    exit()

# ------------------------------------------------------------------------------
# Create the Chrono system
# ------------------------------------------------------------------------------
print("Creating Chrono system...")
sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set solver parameters (adjust if needed for stability/performance)
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)

# ------------------------------------------------------------------------------
# Create the HMMWV vehicle
# ------------------------------------------------------------------------------
print("Creating HMMWV vehicle...")
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChFrameD(init_loc, init_rot))
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(step_size) # Step size for tire dynamics

# Set visualization for all components
hmmwv.SetChassisVisualizationType(vis_type_chassis)
hmmwv.SetSuspensionVisualizationType(vis_type_suspension)
hmmwv.SetSteeringVisualizationType(vis_type_steering)
hmmwv.SetWheelVisualizationType(vis_type_wheel)
hmmwv.SetTireVisualizationType(vis_type_tire)

hmmwv.Initialize()

# Get a handle to the vehicle's powertrain system
powertrain = hmmwv.GetPowertrain()

# ------------------------------------------------------------------------------
# Create the terrain
# ------------------------------------------------------------------------------
print(f"Creating custom mesh terrain from: {terrain_mesh_coll} (collision) and {terrain_mesh_vis} (visual)")
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT), # Position and rotation of the patch
                         terrain_mesh_coll, # Collision OBJ
                         terrain_mesh_vis,  # Visualization OBJ
                         0.8, 0.05)         # Friction coefficient, restitution coefficient (example values)

patch.SetContactMaterial(0.9, 0.01, 2e7, 0.3) # mu, cr, Y, nu (for SMC)
terrain.Initialize()

# ------------------------------------------------------------------------------
# Create the Irrlicht application for visualization
# ------------------------------------------------------------------------------
print("Creating Irrlicht visualization...")
app = irr.ChIrrApp(hmmwv.GetSystem(), "HMMWV on Custom Mesh Terrain", irr.dimension2du(1280, 720))
app.SetSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(hmmwv.GetChassisBody(), 6.0, 0.5) # Target, distance, height
app.SetTimestep(step_size) # Link Irrlicht's step to simulation step

# Bind visualization assets
app.AssetBindAll()
app.AssetUpdateAll()

# ------------------------------------------------------------------------------
# Create the interactive driver system
# ------------------------------------------------------------------------------
print("Creating interactive driver...")
driver = veh.ChIrrGuiDriver(app)

# Set the time response for steering and throttle keyboard inputs.
# NOTE: this is not exact, since we do not render quite at the specified FPS.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# ------------------------------------------------------------------------------
# Simulation loop
# ------------------------------------------------------------------------------
print("\nStarting simulation... Press ESC to exit.")
print("Controls:")
print("  W/S: Throttle/Brake (forward/reverse)")
print("  A/D: Steer Left/Right")
print("  Space: Brake")
print("  Z: Toggle clutch (if manual transmission)")


# Number of simulation steps between two 3D view render frames
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while app.GetDevice().run():
    time = hmmwv.GetSystem().GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs) # For on-screen display

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    
    # Render scene
    if step_number % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192)) # Background color
        app.DrawAll()
        app.EndScene()
        
        # Print vehicle speed (optional)
        # speed_mps = hmmwv.GetVehicle().GetSpeed()
        # speed_kmh = speed_mps * 3.6
        # print(f"Time: {time:.2f} s, Speed: {speed_kmh:.2f} km/h")


    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)
    step_number += 1

del app # Important to close Irrlicht window gracefully
print("Simulation ended.")