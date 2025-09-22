import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os # For robust data path handling

# Attempt to set ChronoDataPath if not already set by environment variable
# This is a more robust way than the original script's redundant call.
if 'CHRONO_DATA_DIR' not in os.environ:
    chrono.SetChronoDataPath(chrono.GetChronoDataPath()) # Fallback to whatever GetChronoDataPath finds
# Ensure CHRONO_DATA_PATH is valid for vehicle and terrain assets
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle', ''))

# Changed initial vehicle location
initLoc = chrono.ChVector3d(-15, 0, 2.7) # Z adjusted for SCM terrain max height + clearance
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
# Chassis collision geometry is not strictly necessary for SCM interaction if using track shoes
chassis_collision_type = veh.CollisionType_NONE

# SCM Deformable Terrain parameters
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction
# SCM Heightmap parameters
hmap_file = veh.GetDataFile("terrain/height_maps/slope.png")
hmap_hMin = 0.0  # Min height in input image (scaled)
hmap_hMax = 2.0  # Max height in input image (scaled) - vehicle Z starts above this
scm_div_x = 100   # Number of SCM patches in X (terrainLength / 1.0m patches)
scm_div_y = 100   # Number of SCM patches in Y (terrainWidth / 1.0m patches)

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1) # Offset from vehicle reference frame

# Contact method
contact_method = chrono.ChContactMethod_SMC
# contact_vis = False # Not used in this script version

# Simulation step sizes
step_size = 5e-4
tire_step_size = step_size # For wheeled vehicles; M113 doesn't use 'tire_step_size' directly for SCM

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the MAN vehicle, set parameters, and initialize
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN) # Options: SINGLE_PIN, IDEAL_SPRING
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)     # Two-sided Driveline (BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Set chassis collision type (important for interaction with SCM)
# vehicle.SetChassisCollisionType(chassis_collision_type) # Already NONE in vars

# Important: Set collision system for the main ChSystem
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the SCM deformable terrain
terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())

# Set SCM soil parameters (example: sandy-loam)
terrain.SetSoilParameters(
    2e6,  # Bekker Kphi (Pa/m^(n+1))
    0,    # Bekker Kc (Pa/m^n)
    1.1,  # Bekker n exponent (-)
    0,    # Mohr cohesive limit (Pa)
    30,   # Mohr friction limit (degrees)
    0.01, # Janosi shear coefficient J (m)
    4e7,  # Elastic K (Pa/m)
    3e4   # Damping C (Pa.s/m)
)
# terrain.SetBulldozingFlow(True)       # Not available in all PyChrono versions or might need specific setup
# terrain.SetYieldStressRatio(0.4)      # For Drucker-Prager based models
terrain.SetWaitonData(False) # If true, SCM will wait for data from external source (not used here)

# Initialize SCM terrain using a height map
terrain.Initialize(
    hmap_file,      # Heightmap PNG file
    terrainLength,  # Terrain length (X direction)
    terrainWidth,   # Terrain width (Y direction)
    hmap_hMin,      # Minimum height value from map
    hmap_hMax,      # Maximum height value from map
    scm_div_x,      # Number of divisions in X
    scm_div_y       # Number of divisions in Y
)

# Attempt to set the SCM terrain texture to dirt
# SCMDeformableTerrain creates a ChTriangleMeshShape attached to its "ground" body.
# We access this shape and apply a texture.
scm_visual_asset_found = False
if terrain.GetGroundBody().GetVisualModel() and terrain.GetGroundBody().GetVisualModel().GetShapes():
    # The visual asset is a tuple (asset, transform)
    mesh_shape_asset_tuple = terrain.GetGroundBody().GetVisualModel().GetShapes()[0]
    if mesh_shape_asset_tuple:
        mesh_shape_asset = mesh_shape_asset_tuple[0] # Get the asset itself
        # Check if it's a ChTriangleMeshShape (or has SetTexture)
        if isinstance(mesh_shape_asset, chrono.ChTriangleMeshShape):
            # Scale texture to repeat e.g. every 2m. If terrain is 100m, 50 repeats.
            texture_repeats_u = terrainLength / 2.0
            texture_repeats_v = terrainWidth / 2.0
            mesh_shape_asset.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), texture_repeats_u, texture_repeats_v)
            scm_visual_asset_found = True
            print("SCM terrain texture set to dirt.jpg")

if not scm_visual_asset_found:
    print("Warning: Could not set texture on SCM terrain visual asset. Using color.")
    terrain.SetColor(chrono.ChColor(0.47, 0.39, 0.26))  # Dirt-like brown color


# Create the vehicle Irrlicht interface
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 SCM Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5) # Point to follow, chase distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time) # Will be overridden for throttle
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Solver and integrator settings
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
# For SCM, BARZILAIBORWEIN is fast but may struggle with stability/accuracy sometimes.
# Consider MINRES or APGD if issues arise.
# vehicle.GetSystem().SetSolverMaxIterations(100) # Example for BARZILAIBORWEIN

# Output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
step_number = 0
# vehicle.GetVehicle().EnableRealtime(True) # Commented out for SCM performance/stability

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs (for steering and braking from keyboard)
    driver_inputs = driver.GetInputs()
    # Hard-code the throttle value to 0.8
    driver_inputs.m_throttle = 0.8

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time) # SCM terrain synchronization
    vehicle.Synchronize(time, driver_inputs) # Pass modified inputs
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size) # SCM terrain advance
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1