import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# ------------------------------------------------------------------------------
# Set Chrono data paths
# ------------------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

# ------------------------------------------------------------------------------
# Simulation parameters
# ------------------------------------------------------------------------------
# Simulation step size
step_size = 1.0 / 50.0  # 50 FPS

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0.7, 0)  # x, y (height), z
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  # Identity quaternion (no rotation)

# Contact method
contact_method = chrono.ChContactMethod_NSC # Or ChContactMethod_SMC

# Tire model
# FEDA has its own default tire models (FEDA_SimpleTire, FEDA_PacejkaTire, etc.)
# We will use its default for this example which is typically a TMeasy-like variant.
# You can explicitly set it:
# tire_model = veh.TireModelType_RIGID
# tire_model = veh.TireModelType_TMEASY 
# tire_model = veh.TireModelType_PACEJKA
# tire_model = veh.TireModelType_Fiala (if available for FEDA)

# Rigid terrain dimensions and texture
terrain_height = 0  # y-level of the terrain
terrain_size_x = 200 # meters
terrain_size_z = 200 # meters
terrain_texture_file = veh.GetDataFile("terrain/textures/tile4.jpg") # Example texture
terrain_texture_scale_x = 200 # Texture repetitions
terrain_texture_scale_z = 200

# Camera setup
camera_chase_track_point = chrono.ChVectorD(0.0, 0.0, 0.0) # Point on vehicle to track
camera_chase_distance = 8.0 # Distance from track point
camera_chase_height = 1.5   # Height above track point

# ------------------------------------------------------------------------------
# Create the Chrono system
# ------------------------------------------------------------------------------
print("Creating Chrono system...")
if contact_method == chrono.ChContactMethod_NSC:
    sys = chrono.ChSystemNSC()
    print("Using NSC contact method.")
elif contact_method == chrono.ChContactMethod_SMC:
    sys = chrono.ChSystemSMC()
    print("Using SMC contact method.")
else:
    raise ValueError("Invalid contact method specified.")

sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # Or other solvers like Type_PSOR
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)

# ------------------------------------------------------------------------------
# Create the FEDA vehicle
# ------------------------------------------------------------------------------
print("Creating FEDA vehicle...")
feda = veh.FEDA()
feda.SetContactMethod(contact_method)
feda.SetChassisFixed(False) # Vehicle is not fixed to ground
feda.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
# feda.SetTireType(tire_model) # Uncomment to explicitly set tire model
feda.SetTireStepSize(step_size) # Important for tire dynamics
feda.Initialize()

# Set visualization type for all vehicle parts to MESH
print("Setting MESH visualization for vehicle parts...")
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH) # FEDA has suspension meshes
feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)   # FEDA has steering meshes
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH) # Or PRIMITIVES if tire mesh is not desired/available

vehicle = feda.GetVehicle()
chassis_body = feda.GetChassisBody()

# ------------------------------------------------------------------------------
# Create the rigid terrain
# ------------------------------------------------------------------------------
print(f"Creating rigid terrain with texture: {terrain_texture_file}")
terrain = veh.RigidTerrain(sys)

# Define material properties for the terrain patch
if contact_method == chrono.ChContactMethod_NSC:
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
else: # SMC
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7) # Example value

# Add a patch of terrain
# Using CSYSNORM for a flat plane at y=terrain_height, normal pointing up (Y_AXIS)
patch = terrain.AddPatch(patch_mat,
                         chrono.CSYSNORM, # Plane normal to Y at origin, then shifted by terrain_height
                         terrain_size_x, terrain_size_z, terrain_height,
                         False, # Don't use a mesh file for geometry, use dimensions
                         terrain_texture_file, "terrain_texture",
                         0.02 # sweeping radius for collision (thickness)
                         )

patch.SetTexture(terrain_texture_file, terrain_texture_scale_x, terrain_texture_scale_z)
patch.GetGroundBody().GetVisualShape(0).SetTexture(terrain_texture_file, terrain_texture_scale_x, terrain_texture_scale_z)
terrain.Initialize()

# ------------------------------------------------------------------------------
# Create the Irrlicht visualization application
# ------------------------------------------------------------------------------
print("Creating Irrlicht application...")
# Create the Irrlicht application with a specific window size
app = veh.ChWheeledVehicleIrrApp(vehicle, "FEDA on Rigid Terrain", irr.dimension2du(1280, 720))

# Set up the camera
# The ChWheeledVehicleIrrApp sets a default chase camera. We can adjust its parameters.
# The track point is relative to the chassis reference frame.
app.SetChaseCamera(camera_chase_track_point, camera_chase_distance, camera_chase_height)
# To set a fixed camera position, you would do something like:
# app.SetCameraPosition(irr.vector3df(10, 5, 15), irr.vector3df(0, 0, 0))

app.SetTimestep(step_size)
app.AddTypicalLights() # Add default lighting
app.SetSkyBox() # Add a default skybox
app.AssetBindAll()
app.AssetUpdateAll()


# ------------------------------------------------------------------------------
# Create the interactive driver system
# ------------------------------------------------------------------------------
print("Creating interactive driver...")
driver = veh.ChInteractiveDriverIRR(app)

# Set the time response for steering and throttle/ブレーキ controls
driver.SetSteeringDelta(0.04)  # Rate of steering change per input
driver.SetThrottleDelta(0.04)  # Rate of throttle change per input
driver.SetBrakingDelta(0.1)   # Rate of braking change per input
driver.Initialize()

print("\nControls:")
print("Steering: A/D")
print("Throttle: W")
print("Braking: S")
print("Camera Zoom: Z/X")
print("Camera Orbit: LMB drag")
print("------------------------------------------------------------------------------")

# ------------------------------------------------------------------------------
# Simulation loop
# ------------------------------------------------------------------------------
time = 0.0
while app.GetDevice().run():
    current_time = sys.GetChTime()

    # Update HUD elements
    app.BeginScene(True, True, irr.SColor(255, 140, 160, 190)) # Clear screen, Z buffer, set color
    app.DrawAll() # Draw Chrono items

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize and advance systems
    driver.Synchronize(current_time)
    vehicle.Synchronize(current_time, driver_inputs, terrain) # Pass driver inputs and terrain
    terrain.Synchronize(current_time) # Important if terrain moves or deforms
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs) # Synchronize app with driver info

    # Advance simulation for one step
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size) # Important if terrain dynamics are involved
    app.Advance(step_size)

    # Advance the system by one step
    sys.DoStepDynamics(step_size)

    app.EndScene()

    # Check if simulation is stuck or vehicle overturned (optional)
    if chassis_body.GetPos().y < -1 : # Example condition: vehicle fell through or severely overturned
        print(f"Vehicle Y position {chassis_body.GetPos().y} too low. Ending simulation.")
        break
    if abs(chassis_body.GetRot().Q_to_Euler123().x) > 1.5 or abs(chassis_body.GetRot().Q_to_Euler123().z) > 1.5 :
        # Note: Q_to_Euler123() x is pitch, z is roll for standard Chrono vehicle frames
        print(f"Vehicle excessively tilted. Ending simulation.")
        # break # Uncomment to stop if vehicle overturns too much

    # Increment time (optional if relying solely on sys.GetChTime())
    time += step_size

del app # Important to release Irrlicht resources
print("Simulation finished.")