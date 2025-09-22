import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os # Often useful for path management, though not strictly needed if CHRONO_DATA_DIR is set

"""
!!!! Set this path before running the demo!
"""
# It's assumed that CHRONO_DATA_DIR is set in the environment,
# or GetChronoDataPath() otherwise provides the correct base path.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method - MODIFIED as per instructions
contact_method = chrono.ChContactMethod_NSC # Changed from SMC to NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 20  # FPS = 50 (Note: original comment said 50, 1/20 = 20 FPS)
                               # Correcting to 1.0 / 50 for 50 FPS, or keeping 1.0/20 for 20 FPS.
                               # Let's keep 1.0/20 as in original script, meaning 20 FPS.

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced() could be another choice here
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

# Set the collision system type (important for both NSC and SMC)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Terrain Creation: MODIFIED ---
# Changed from SCM deformable terrain to RigidTerrain with a single heightmap patch.

# Create the rigid terrain object, associated with the same ChSystem as the vehicle
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Define properties for the heightmap patch based on previous SCM setup
hmap_file = veh.GetDataFile("terrain/height_maps/bump64.bmp")
texture_file = veh.GetDataFile("terrain/textures/dirt.jpg")
patch_size_x = 40.0  # Corresponds to '40' in SCM Initialize
patch_size_y = 40.0  # Corresponds to '40' in SCM Initialize
h_min = -1.0         # Corresponds to '-1' in SCM Initialize
h_max = 1.0          # Corresponds to '1' in SCM Initialize
texture_scale_x = 6.0 # From SCM terrain.SetTexture
texture_scale_y = 6.0 # From SCM terrain.SetTexture

# Create a contact material for the terrain patch (NSC)
patch_material = chrono.ChContactMaterialNSC()
patch_material.SetFriction(0.9)  # Typical friction value for NSC
# For NSC, restitution is often 0 unless specific bouncing is desired
# patch_material.SetRestitution(0.01)

# Add the heightmap patch to the terrain
# The patch is centered at the origin (CSYSNORM)
# The heightmap values (0-255) are mapped to the range [h_min, h_max]
patch = terrain.AddPatch(patch_material,
                         chrono.CSYSNORM,  # Position and orientation (centered at origin)
                         hmap_file,
                         "heightmap_terrain_patch",  # A name for the mesh
                         patch_size_x,
                         patch_size_y,
                         h_min,
                         h_max,
                         0.0) # sweep_sphere_radius (optional, default 0)

# Set the texture for the patch
patch.SetTexture(texture_file, texture_scale_x, texture_scale_y)

# Optionally, set a color for the patch (e.g., if textures are not displayed)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.3)) # A brownish color

# Initialize the terrain system (after all patches are added)
terrain.Initialize()

# --- End of Terrain Creation Modification ---

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Rigid Terrain Demo') # Updated title
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())
# It's good practice to attach the terrain to the visualization system as well
# although RigidTerrain patches are often automatically handled if they have visual assets.
# For explicit control:
# vis.AttachTerrain(terrain) # Might not be strictly necessary for RigidTerrain patches with viz.

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

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
# render_frame = 0 # This variable was defined but not used in the original loop logic

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # render_frame += 1 # If you need to count rendered frames

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time) # RigidTerrain also needs synchronization
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size) # RigidTerrain also needs advancing
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# Cleanup (optional, but good practice if you were managing resources manually)
# vis.EndScene() # Already in loop
# No explicit destructor calls needed for these PyChrono objects usually.