import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import os
import math

# -----------------------------------------------------------------------------
# Set Chrono data directory
# -----------------------------------------------------------------------------
# Attempt to set the Chrono data path.
# The data directory contains models (meshes, textures, CAD parts, etc.).
try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
    print(f"Using CHRONO_DATA_DIR: {os.environ['CHRONO_DATA_DIR']}")
except KeyError:
    # If CHRONO_DATA_DIR is not set, try a common relative path.
    # This might need adjustment based on your PyChrono installation structure.
    print("CHRONO_DATA_DIR environment variable not set.")
    default_data_path = os.path.join(os.path.dirname(chrono.__file__), '..', '..', '..', 'data', '')
    chrono.SetChronoDataPath(default_data_path)
    print(f"Attempting to use default data path: {os.path.abspath(default_data_path)}")

# Ensure the vehicle data path is also set (it usually inherits from Chrono's data path)
vehicle.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
print(f"Vehicle data path: {vehicle.GetDataPath()}")

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
# Simulation step size
step_size = 2e-3  # Corresponds to 500 Hz simulation frequency

# Initial vehicle position and orientation
init_loc = chrono.ChVectorD(0, 0.7, 0)  # x, y (height), z
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # No initial rotation

# Terrain parameters
terrain_height = 0.0
terrain_size_x = 200.0  # meters
terrain_size_y = 200.0  # meters
terrain_friction = 0.9
terrain_restitution = 0.01
terrain_young_modulus = 2e7  # Pa
terrain_poisson_ratio = 0.3

# Visualization settings
camera_chase_track_point = chrono.ChVectorD(0.0, 0.0, 0.0) # Point on vehicle to track
camera_chase_distance = 8.0 # Distance from track point
camera_chase_height = 1.5   # Height of camera relative to track point

# -----------------------------------------------------------------------------
# 1. Initialize PyChrono environment and core components
# -----------------------------------------------------------------------------
# Create a Chrono system (NSC: Non-Smooth Contact)
# For SMC (Smooth Contact), use chrono.ChSystemSMC()
# and adjust contact material properties accordingly.
my_system = chrono.ChSystemNSC()
my_system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# Set solver settings if needed (defaults are often fine for NSC)
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # A good general-purpose solver
my_system.SetSolverMaxIterations(150)
my_system.SetMaxPenetrationRecoverySpeed(4.0)

# -----------------------------------------------------------------------------
# 2. Add the required physical systems and objects
# -----------------------------------------------------------------------------

# Create the M113 vehicle
# Options:
# - Fixed: False (not rigidly attached to ground)
# - Driveline: SIMPLE (can be SHAFT for more detailed model)
# - BrakeType: SIMPLE
# - EngineModel: SIMPLE_MAP (can be SHAFTS for more detail)
# - TransmissionModel: AUTOMATIC_SIMPLE_MAP
# - ChassisCollisionType: NONE, PRIMITIVES, HULLS, MESH
#   NONE is often used when primary interaction is tracks-terrain
my_m113 = vehicle.M113_Vehicle(
    fixed=False,
    driveline_type=vehicle.DrivelineTypeWV.SIMPLE,
    brake_type=vehicle.BrakeType.SIMPLE,
    engine_model=vehicle.EngineModelType.SIMPLE_MAP,
    transmission_model=vehicle.TransmissionModelType.AUTOMATIC_SIMPLE_MAP,
    system=my_system,
    chassis_collision_type=vehicle.ChassisCollisionType.NONE
)

# Initialize the vehicle at the specified position and orientation
my_m113.Initialize(chrono.ChCoordsysD(init_loc, init_rot))

# Set visualization for vehicle components (chassis, wheels, tracks)
my_m113.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
my_m113.SetSuspensionVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_m113.SetSteeringVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_m113.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
my_m113.SetTrackShoeVisualizationType(vehicle.VisualizationType_MESH)


# Create the rigid terrain
terrain = vehicle.RigidTerrain(my_system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)
# For SMC terrain, you would use chrono.ChMaterialSurfaceSMC() and set Young's modulus, Poisson's ratio, etc.
# patch_mat_smc = chrono.ChMaterialSurfaceSMC()
# patch_mat_smc.SetYoungModulus(terrain_young_modulus)
# patch_mat_smc.SetPoissonRatio(terrain_poisson_ratio)
# patch_mat_smc.SetFriction(terrain_friction)
# patch_mat_smc.SetRestitution(terrain_restitution)

# Initialize terrain with a single large patch
# Parameters: height, sizeX, sizeY, (optional) center_x, center_z
# For NSC:
terrain.AddPatch(patch_mat,
                 chrono.CSYSNORM, # By default, plane is at Y=0, normal along Y
                 terrain_size_x, terrain_size_y)

# For SMC, it would be:
# terrain.AddPatch(patch_mat_smc, chrono.CSYSNORM, terrain_size_x, terrain_size_y)

terrain.Initialize()

# Set terrain visualization color
terrain_asset = terrain.GetGroundBody().GetAssets()[0]
visual_asset = chrono.CastToChVisualization(terrain_asset)
visual_asset.material_list[0].SetKdTexture(chrono.GetChronoDataPath() + "textures/concrete.jpg")
# To set a simple color:
# visual_asset.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

# -----------------------------------------------------------------------------
# 3. Set necessary default parameters (already done for vehicle and terrain)
#    Forces are typically applied via engine/driver models or external ChForce
#    Interactions are handled by the contact system and material properties
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Create the Irrlicht visualization system and driver
# -----------------------------------------------------------------------------

# Create the Irrlicht application for visualization
# This ChVehicleIrrApp also handles the GUI driver inputs if one is attached.
vis_app = vehicle.ChVehicleIrrApp(my_m113, "M113 Simulation")
vis_app.SetSkyBox()
vis_app.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
vis_app.SetChaseCamera(camera_chase_track_point, camera_chase_distance, camera_chase_height)
vis_app.SetTimestep(step_size) # Important for the an internal timer in the IrrApp

# Create an interactive driver system (GUI inputs for throttle, steering, braking)
# This driver is then attached to the visualization application.
driver = vehicle.ChIrrGuiDriver(vis_app)

# Set the time response for steering and throttle inputs.
# Smaller time means faster response.
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)

# Attach the driver to the vehicle
my_m113.SetDriver(driver) # The vehicle needs to know about its driver

# Finalize Irrlicht application setup
vis_app.AssetBindAll()
vis_app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
# Create a real-time timer
rt_timer = chrono.ChRealtimeStepTimer()

print("Simulation started. Close the Irrlicht window to end.")

while vis_app.Run():
    time = my_system.GetChTime()

    # Update HUD elements
    vis_app.BeginScene(True, True, irr.SColor(255, 140, 160, 190))
    vis_app.DrawAll() # Render visualization
    # Add custom text or overlays here if needed using vis_app.GetIrrlichtDevice().getVideoDriver()....
    # Example: vis_app.GetIrrlichtDevice().getVideoDriver().draw2DImage(...)
    #          vis_app.GetIrrlichtDevice().getGUIEnvironment().getBuiltInFont().draw(...)
    
    # Get driver inputs (steering, throttle, braking)
    # These are automatically updated by ChIrrGuiDriver based on keyboard/GUI
    driver_inputs = driver.GetInputs()

    # Synchronize systems
    # For ChIrrGuiDriver, Synchronize() gets inputs from the GUI
    # For the vehicle, Synchronize() applies driver inputs to the vehicle's powertrain/steering
    driver.Synchronize(time)
    my_m113.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time) # Important if terrain has moving parts or needs updates
    vis_app.Synchronize(my_m113.GetReportContactRecord()) # For contact forces visualization

    # Advance simulation
    my_m113.Advance(step_size)
    terrain.Advance(step_size) # Important if terrain has moving parts
    vis_app.Advance(step_size) # Advance visualization animation time
    my_system.DoStepDynamics(step_size) # Perform the physics integration step

    vis_app.EndScene()

    # Spin the real-time timer to maintain real-time execution
    rt_timer.Spin(step_size)

print("Simulation ended.")
del vis_app # Important to properly close Irrlicht window and resources
del my_system # Clean up Chrono system