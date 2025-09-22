import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os # Added for os.path.join, though not strictly used in final version to keep original style

# Ensure CHRONO_DATA_DIR is set in your environment variables
# Example: export CHRONO_DATA_DIR=/path/to/chrono/data
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '')) # More robust way to get data path
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5) # Start at origin, will be on the first patch
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY # TMEASY is good for uneven terrain

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1) # Gator specific tracking point

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False # Set to True to visualize contacts

# Simulation step sizes
step_size = 2e-3 # Adjusted for potentially more complex contacts
tire_step_size = 1e-3 # TMEasy might benefit from smaller step

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set the collision system type for the entire system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7) # Optional: stiffness

terrain = veh.RigidTerrain(vehicle.GetSystem())

# Define dimensions and parameters for terrain patches
patch_length = 40.0  # Length of each flat patch
patch_width = 15.0   # Width of all patches
terrain_base_z = -0.2 # Base Z level for flat terrain patches, ensuring vehicle starts above

# Patch 1: Flat terrain with texture 1
p1_center_x = patch_length / 2.0
patch1_csys = chrono.ChCoordsysd(chrono.ChVector3d(p1_center_x, 0, terrain_base_z), chrono.QUNIT)
patch1 = terrain.AddPatch(patch_mat, patch1_csys, patch_length, patch_width)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), patch_length / 2, patch_width / 2) # Adjusted texture scaling
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: Flat terrain with texture 2 (and a bump)
p2_center_x = patch_length * 1.5
patch2_csys = chrono.ChCoordsysd(chrono.ChVector3d(p2_center_x, 0, terrain_base_z), chrono.QUNIT)
patch2 = terrain.AddPatch(patch_mat, patch2_csys, patch_length, patch_width)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), patch_length / 2, patch_width / 2)
patch2.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

# Add a bump to Patch 2
bump_x_dim = 0.8  # Length of bump along X
bump_y_dim = patch_width * 0.6 # Width of bump across Y
bump_z_dim = 0.25 # Height of bump
bump_center_x = p2_center_x # Place it in the middle of Patch 2
bump_center_y = 0
bump_center_z = terrain_base_z + bump_z_dim / 2.0

bump = chrono.ChBodyEasyBox(bump_x_dim, bump_y_dim, bump_z_dim,
                             1000,    # Density (not really used if fixed)
                             True,    # Enable visualization
                             True,    # Enable collision
                             patch_mat) # Use same material as terrain
bump.SetPos(chrono.ChVector3d(bump_center_x, bump_center_y, bump_center_z))
bump.SetBodyFixed(True)
bump.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg")) # Match underlying patch
vehicle.GetSystem().Add(bump)


# Patch 3: Heightmap terrain for gradability
hm_length = 50.0  # Length of heightmap patch
hm_width = patch_width   # Width of heightmap patch
hm_min_height = 0.0 # Min height relative to CSYS Z
hm_max_height = 2.0 # Max height relative to CSYS Z (can create a 2m hill)
p3_center_x = patch_length * 2.0 + hm_length / 2.0

# CSYS for heightmap: Z position is crucial.
# The heightmap data (min_h, max_h) is applied relative to this CSYS.
# So, if CSYS z = terrain_base_z, lowest point of HM will be at terrain_base_z.
patch3_csys = chrono.ChCoordsysd(chrono.ChVector3d(p3_center_x, 0, terrain_base_z), chrono.QUNIT)
heightmap_file = veh.GetDataFile("terrain/height_map.bmp") # Standard Chrono heightmap
patch3 = terrain.AddPatch(patch_mat,
                          patch3_csys,
                          heightmap_file,    # Heightmap file
                          "heightmap_mesh",  # Name for the mesh
                          hm_length,         # Size X of the patch
                          hm_width,          # Size Y of the patch
                          hm_min_height,     # Min height value
                          hm_max_height,     # Max height value
                          0,                 # No data type for this typical BMP
                          0.0)               # Swept sphere radius (0 for mesh)

# Set visualization for the heightmap patch
# Note: RigidTerrain::AddPatch for heightmap automatically creates a ChTriangleMeshShape
# We can access it if needed, but usually, its material/texture is set via the patch itself.
# For heightmap, the texture is applied to the generated mesh.
patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), hm_length / 5, hm_width / 5) # Texture for the heightmap
patch3.SetColor(chrono.ChColor(0.5, 0.7, 0.5)) # Greenish color

# Patch 4: Flat terrain with texture 3
p4_center_x = patch_length * 2.0 + hm_length + patch_length / 2.0
patch4_csys = chrono.ChCoordsysd(chrono.ChVector3d(p4_center_x, 0, terrain_base_z), chrono.QUNIT)
patch4 = terrain.AddPatch(patch_mat, patch4_csys, patch_length, patch_width)
patch4.SetTexture(veh.GetDataFile("terrain/textures/rock.jpg"), patch_length / 3, patch_width / 2)
patch4.SetColor(chrono.ChColor(0.5, 0.4, 0.3))

terrain.Initialize()

# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle on Diverse Terrain')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) # Camera distance and height
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
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

print("Chrono data path: " + chrono.GetChronoDataPath())
print("Vehicle data path: " + veh.GetDataPath())


while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

del vehicle
del driver
# No explicit del vis needed for Irrlicht usually, it handles its own closure