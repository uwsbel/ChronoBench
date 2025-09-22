import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.scm as scm
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
step_size = 5e-4  # SCM typically requires smaller step sizes

# End time
time_end = 100

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(-70, 1.0, -70) # Start on a relatively flat part of the chosen map
initRot = chrono.ChQuaternionD(1, 0, 0, 0) # No initial rotation

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH # Use MESH for better SCM interaction visuals
tire_vis_type = veh.VisualizationType_MESH

# Type of tire model (RIGID, PACEJKA, TMEASY, FIALA, PAC89, PAC02)
# For SCM, RIGID or RIGID_MESH are typically used for the tire *model*,
# as SCM handles the tire-terrain interaction.
tire_model = veh.TireModelType_RIGID

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.0) # Relative to HMMWV chassis CoG

# SCM Patch dimensions
patch_dim_x = 160.0  # x-dimension (length) of SCM patch
patch_dim_y = 160.0  # y-dimension (width) of SCM patch

# SCM Bulldozing
enable_bulldozing = True

# SCM Plot type for soil properties visualization
# PLOT_NONE, PLOT_PRESSURE, PLOT_PRESSURE_YELD, PLOT_SINKAGE, PLOT_SINKAGE_ELASTIC, PLOT_SINKAGE_PLASTIC
plot_type = scm.SCMDeformableTerrain.PLOT_SINKAGE
# Plot output range
plot_output_min = 0
plot_output_max = 0.3 # Max sinkage to visualize

# -----------------------------------------------------------------------------
# Create the Chrono system
# -----------------------------------------------------------------------------
# Use ChSystemSMC for SCM, as it handles contact with deformable bodies well.
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set the solver precision (improves stability with SCM)
system.SetMaxItersSolverSpeed(150)
system.SetMaxItersSolverStab(150)
# system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # Optional: try different solvers

# -----------------------------------------------------------------------------
# Create the HMMWV vehicle
# -----------------------------------------------------------------------------
# Create the HMMWV_Reduced vehicle
my_hmmwv = veh.HMMWV_Reduced(system)
my_hmmwv.SetContactMethod(chrono.ChMaterialSurface.SMC) # Ensure vehicle uses SMC
my_hmmwv.SetChassisFixed(False)
my_hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE) # Or AUTOMATIC_SIMPLEMAP
my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD) # All-Wheel Drive
my_hmmwv.SetTireType(tire_model)
my_hmmwv.SetTireStepSize(step_size) # Important for tire model stability

# Initialize the HMMWV
my_hmmwv.Initialize()

# Set visualization type for vehicle components
my_hmmwv.SetChassisVisualizationType(chassis_vis_type)
my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
my_hmmwv.SetSteeringVisualizationType(steering_vis_type)
my_hmmwv.SetWheelVisualizationType(wheel_vis_type)
my_hmmwv.SetTireVisualizationType(tire_vis_type)

# Get the HMMWV_Vehicle object for easier access to vehicle components
vehicle = my_hmmwv.GetVehicle()

# -----------------------------------------------------------------------------
# Create the SCM deformable terrain
# -----------------------------------------------------------------------------
# Create an SCM deformable terrain patch
terrain = scm.SCMDeformableTerrain(system)

# Configure the SCM terrain
# Bekker-Wong parameters (typical values for sand)
# Kphi, Kc, n, Cohesion (Pa), Friction Angle (deg), Janosi K (m), Elastic K (Pa/m), Damping C (Pa*s/m)
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                          0,     # Bekker Kc
                          1.1,   # Bekker n exponent
                          20e3,  # Mohr cohesive limit (Pa) default: 0
                          30.0,  # Mohr friction limit (degrees) default: 30
                          0.01,  # Janosi shear K (m) default: 0.01
                          1.2e8, # Elastic K (Pa/m) default: 4e7
                          3e4)   # Damping C (Pa s/m) default: 3e4

# Enable bulldozing effects (optional, can be computationally expensive)
terrain.SetBulldozingFlow(enable_bulldozing)
if enable_bulldozing:
    terrain.SetBulldozingParameters(55,   # Angle of erosion
                                    1,    # Factor of erosion
                                    0.2,  # Limit of erosion
                                    1)    # Limit of accumulation

# Set plot type for SCM visualization (shows soil property on the deformed mesh)
terrain.SetPlotType(plot_type, plot_output_min, plot_output_max)

# Initialize the SCM terrain using a height map
# Height map file (ensure this path is correct and the file exists)
# Using slope.bmp as a common example, test_HMMWV_SCM.bmp is also good if available
heightmap_file = chrono.GetChronoDataFile("terrain/height_maps/slope.bmp")
# heightmap_file = veh.GetDataFile("terrain/height_maps/test_HMMWV_SCM.bmp") # Alternative

# Length and width of the terrain patch in meters
terrain_length = patch_dim_x
terrain_width = patch_dim_y

# Min and max height of the terrain from the height map
min_height = 0.0
max_height = 2.0 # Adjust based on the height map used (slope.bmp is relatively flat)

# Up direction (Y is up for SCM) and resolution
# Note: SCM terrain is typically defined in the X-Z plane with Y as height
# The up_dir (0,-1,0) might seem counter-intuitive, but it's related to how
# Chrono's SCM processes the heightmap image (image y-axis vs world y-axis).
# For standard Y-up Chrono systems, (0,1,0) for up_dir is also common and often more direct.
# Let's try with (0,1,0) which aligns with typical Y-up coordinates.
# If terrain appears inverted, (0,-1,0) might be needed depending on heightmap format.
up_dir = chrono.ChVectorD(0, 1, 0)
resolution_x = 0.1  # meters per pixel in x
resolution_y = 0.1  # meters per pixel in y

terrain.Initialize(heightmap_file,
                   terrain_length, terrain_width,
                   min_height, max_height,
                   up_dir,
                   resolution_x, resolution_y)

# Set the SCM terrain visualization assets
terrain.GetMesh().SetWireframe(False) # Render as solid surface


# -----------------------------------------------------------------------------
# Create the Irrlicht visualization application
# -----------------------------------------------------------------------------
vis_app = irr.ChIrrApp(system, "HMMWV on SCM Deformable Terrain", irr.dimension2du(1280, 720))
vis_app.SetTimestep(step_size) # Visualization timestep matches simulation

# Add vehicle to the visualization manager
# Note: SCMTerrain visualization is handled by the SCM module itself,
# it automatically adds its visualization assets to the system.
# We explicitly add the vehicle.
# vis_app.AddTypicalLights()
vis_app.AddLight(irr.SLight(chrono.ChVectorD(100,100,100), chrono.ChColor(0.8,0.8,0.8),300))
vis_app.AddLight(irr.SLight(chrono.ChVectorD(-100,100,-100), chrono.ChColor(0.8,0.8,0.8),300))


# Attach the HMMWV vehicle's Irrlicht assets to the scene.
# This will automatically add the chassis, wheels, and tires to the Irrlicht scene.
# The my_hmmwv.SetChassisVisualizationType etc. calls configure *what* to visualize.
# Binding assets makes them appear.
vis_app.AssetBindAll()
vis_app.AssetUpdateAll()


# Set up the camera
# The ChChaseCamera tracks a point on the vehicle's chassis.
# Arguments: (point to track relative to chassis CoM, chase distance, chase height)
vis_app.SetChaseCamera(trackPoint, my_hmmwv.GetChassis().GetBody(), 6.0)
vis_app.SetChaseCameraState(irr.ChChaseCamera.Track) # Ensure it's tracking
vis_app.SetChaseCameraPosition(my_hmmwv.GetChassis().GetBody().TransformPointLocalToParent(trackPoint) + chrono.ChVectorD(-8, 3, 0))
vis_app.SetChaseCameraAngle(-math.pi/8) # Slight downward angle
vis_app.SetChaseCameraMultipliers(1.0, 10.0) # Speed multipliers for camera movement

# -----------------------------------------------------------------------------
# Create the driver system
# -----------------------------------------------------------------------------
# An interactive driver connected to the Irrlicht window
driver = veh.ChIrrGuiDriver(vis_app)

# Set the time response for steering and throttle keyboard inputs.
# Smaller time response means slower keyboard response.
driver.SetSteeringDelta(0.02)  # Adjust for desired sensitivity
driver.SetThrottleDelta(0.04)  # Adjust for desired sensitivity
driver.SetBrakingDelta(0.06)
driver.Initialize()


# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
# Real-time synchronization
# Using ChRealtimeStepTimer to attempt real-time execution
realtime_timer = chrono.ChRealtimeStepTimer()
# realtime_timer.SetVerbose(True) # Optional: print sync info

# Simulation time
time = 0.0

print("Starting simulation...")
print("------------------------------------")
print("Controls:")
print("  Throttle:  W / S")
print("  Steering:  A / D")
print("  Braking:   Spacebar (hold)")
print("  Toggle Camera Mode: C")
print("------------------------------------")


while vis_app.GetDevice().run():
    time = system.GetChTime()

    # End simulation
    if time >= time_end:
        print("Simulation ended.")
        break

    # Get driver inputs
    steering_input = driver.GetSteering()
    throttle_input = driver.GetThrottle()
    braking_input = driver.GetBraking()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time) # SCM terrain needs to be synchronized before vehicle
    my_hmmwv.Synchronize(time, steering_input, throttle_input, braking_input, terrain)
    
    vis_app.BeginScene(True, True, chrono.ChColor(0.2, 0.3, 0.5)) # Clear background, Z-buffer, set color
    vis_app.Render() # Render all visualization assets

    # Draw GUI elements (like driver inputs)
    driver.DrawAll()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    my_hmmwv.Advance(step_size)
    system.DoStepDynamics(step_size) # Core physics step
    # vis_app.Advance(step_size) # Usually not needed if BeginScene/EndScene used explicitly

    vis_app.EndScene()

    # Try to maintain real-time execution
    realtime_timer.Spin(step_size)

# Cleanup
print("Simulation complete.")