import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# -----------------------------------------------------------------------------
# Set Chrono data directory
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
# Simulation step size
step_size = 1.0 / 50.0  # 50 FPS

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)  # x, y, z position
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  # Identity quaternion

# Contact method
contact_method = chrono.ChContactMethod_NSC  # Non-smooth contact

# Tire model: TMEASY
tire_model = veh.TireModelType_TMEASY

# Visualization type for vehicle components
vis_type_chassis = veh.VisualizationType_MESH
vis_type_suspension = veh.VisualizationType_MESH
vis_type_steering = veh.VisualizationType_MESH
vis_type_wheel = veh.VisualizationType_MESH
vis_type_tire = veh.VisualizationType_MESH # TMEASY tires have their own mesh visualization

# Rigid terrain dimensions
terrain_length = 200.0  # size in X direction
terrain_width = 200.0   # size in Y direction
terrain_height = 0.1    # thickness of the terrain box (visual only, collision is plane)
terrain_friction = 0.8

# Custom texture for terrain (ensure this file exists in your Chrono data path)
# Example: using a texture from Chrono's default data
terrain_texture_file = chrono.GetChronoDataFile("vehicle/terrain/textures/rock.jpg")
terrain_texture_scale_x = 20  # How many times the texture repeats in X
terrain_texture_scale_y = 20  # How many times the texture repeats in Y

# -----------------------------------------------------------------------------
# Create the Chrono system
# -----------------------------------------------------------------------------
system = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # A good fast solver
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# -----------------------------------------------------------------------------
# Create the Gator vehicle
# -----------------------------------------------------------------------------
gator = veh.Gator(system)

gator.SetContactMethod(contact_method)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
gator.SetPowertrainType(veh.PowertrainModelType_SHAFTS)  # Simple powertrain
gator.SetDriveType(veh.DrivelineTypeWV_AWD)             # All-wheel drive
gator.SetTireType(tire_model)
gator.SetTireStepSize(step_size) # Step size for tire dynamics
gator.Initialize()

# Set visualization types for the vehicle parts
gator.SetChassisVisualizationType(vis_type_chassis)
gator.SetSuspensionVisualizationType(vis_type_suspension, 0, veh.VisualizationType_PRIMITIVES) # Left, Right (using primitives for leaf springs for clarity)
gator.SetSuspensionVisualizationType(vis_type_suspension, 1, veh.VisualizationType_PRIMITIVES) # Left, Right
gator.SetSteeringVisualizationType(vis_type_steering, 0, veh.VisualizationType_PRIMITIVES) # Left, Right
gator.SetWheelVisualizationType(vis_type_wheel)
gator.SetTireVisualizationType(vis_type_tire)

print("Gator vehicle initialized.")
print("Location:", gator.GetVehicle().GetChassisBody().GetPos())
print("Orientation (Quat):", gator.GetVehicle().GetChassisBody().GetRot())
print("Tire model:", gator.GetTireType())

# -----------------------------------------------------------------------------
# Create the rigid terrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
material = chrono.ChMaterialSurfaceNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChMaterialSurfaceSMC()
material.SetFriction(terrain_friction)
material.SetRestitution(0.01)

# Add a patch of flat terrain
# The ChRigidTerrain.AddPatch function expects:
# material: ChMaterialSurface
# CSYS: Coordinate system of the patch (position and orientation)
# size_x: length
# size_y: width
# MESH (for heightmap/mesh based terrain) or BOX (for simple flat terrain patch)
# If using BOX, you specify thickness for visualization. Collision is always planar.
patch = terrain.AddPatch(material,
                         chrono.CSYSNORM,  # Centered at origin, Z up
                         terrain_length,
                         terrain_width)

patch.SetTexture(terrain_texture_file, terrain_texture_scale_x, terrain_texture_scale_y)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5)) # Base color if texture fails to load

terrain.Initialize()
print("Rigid terrain initialized.")

# -----------------------------------------------------------------------------
# Create the Irrlicht application for visualization
# -----------------------------------------------------------------------------
app_title = "Gator on Rigid Terrain - TMEASY Tires"
app_width = 1280
app_height = 720

application = irr.ChIrrApp(system, app_title, irr.dimension2du(app_width, app_height))
application.SetTimestep(step_size) # Synchronize Irrlicht's timestep with simulation

# Add typical Irrlicht camera and lights
application.AddTypicalLights()

# Set up a chase camera
# Arguments: chassis body, track point, camera distance, camera height
# Track point is relative to chassis CoM
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75) # Point to follow on the vehicle
camera_dist = 8.0 # Distance from track point
camera_height = 1.5 # Height of camera relative to track point
application.SetChaseCamera(gator.GetVehicle().GetChassisBody(), camera_dist, 0.5)
application.SetChaseCameraPosition(gator.GetVehicle().GetChassisBody().TransformPointLocalToParent(trackPoint) + chrono.ChVectorD(0, camera_dist * 0.707 , camera_height))
application.SetChaseCameraLookAt(gator.GetVehicle().GetChassisBody().TransformPointLocalToParent(trackPoint))


# Bind all assets for visualization
application.AssetBindAll()
application.AssetUpdateAll()

# -----------------------------------------------------------------------------
# Create the interactive driver system
# -----------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(application)

# Set the time response for steering and throttle keyboard inputs.
# Smaller time means faster response.
steering_time = 1.0  # time to go from 0 to +1 (or -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

driver.SetSteeringDelta(step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)

driver.Initialize()
print("Interactive driver initialized. Use A/D for steering, W/S for throttle/brake.")

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
# Number of simulation steps for data logging (if any)
# Sim_end_time = 1000

# Initialize simulation time
time = 0

while application.GetDevice().run():
    time = system.GetChTime()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time) # Terrain might have moving parts in other scenarios
    application.Synchronize(driver.GetInputModeAsString(), driver_inputs) # For on-screen messages

    # Advance simulation for one timestep
    driver.Advance(step_size)
    gator.Advance(step_size)
    terrain.Advance(step_size)
    application.AdvanceModel() # Advance the Irrlicht items

    # Render scene
    application.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) # Background color
    application.DrawAll()
    
    # Draw the GUI elements (like driver input display)
    # This needs to be called after DrawAll and before EndScene
    driver.DrawGUI() # If you want to see the driver input levels on screen

    application.EndScene()

    # if (time > Sim_end_time): # Optional: end simulation after a certain time
    #     break

del application # Important to cleanup Irrlicht app and release resources
print("Simulation finished.")