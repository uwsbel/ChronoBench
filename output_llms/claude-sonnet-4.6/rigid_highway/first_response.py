import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os

# =============================================================================
# Initial vehicle parameters
# =============================================================================
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step size
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# =============================================================================
# Create the HMMWV vehicle
# =============================================================================
hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)

hmmwv.SetChassisFixed(False)

hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)

hmmwv.Initialize()

# Set visualization types for all vehicle components
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# =============================================================================
# Create the terrain
# =============================================================================
terrain = veh.RigidTerrain(hmmwv.GetSystem())

# Define material properties based on contact method
if contact_method == chrono.ChContactMethod_NSC:
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
elif contact_method == chrono.ChContactMethod_SMC:
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)

# Check if mesh files exist, otherwise use a flat patch as fallback
col_mesh_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj")
vis_mesh_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_vis.obj")

# Try to use mesh terrain
use_mesh_terrain = os.path.exists(col_mesh_file)

if use_mesh_terrain:
    print("Using mesh terrain: Highway_col.obj / Highway_vis.obj")
    
    # Create mesh patch using collision mesh
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        col_mesh_file,
        False,   # not wireframe
        0.01,    # sphere swept thickness
        False    # no smoothing
    )
    
    # Set visual mesh
    patch.SetMeshVisualization(vis_mesh_file)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
else:
    print("Mesh files not found. Using flat terrain patch as fallback.")
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        200.0, 200.0
    )
    patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# =============================================================================
# Create the vehicle Irrlicht interface
# =============================================================================
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Full - Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.SetHUDLocation(500, 20)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# =============================================================================
# Create the interactive driver system
# =============================================================================
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0   # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0   # time to go from 0 to +1
braking_time = 0.3    # time to go from 0 to +1

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# =============================================================================
# Output directory setup
# =============================================================================
out_dir = chrono.GetChronoOutputPath() + "HMMWV_MESH"
if not os.path.exists(out_dir):
    try:
        os.makedirs(out_dir)
    except:
        pass

# =============================================================================
# Simulation loop
# =============================================================================
print("HMMWV simulation on mesh terrain")
print(f"Contact method: {'SMC' if contact_method == chrono.ChContactMethod_SMC else 'NSC'}")
print(f"Tire model: TMEASY")
print(f"Step size: {step_size}")
print(f"Render FPS: {1.0/render_step_size}")
print()
print("Controls:")
print("  Arrow keys: Steering")
print("  W/S: Throttle/Brake")
print("  Space: Brake")
print("  Q or Esc: Quit")
print()

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
step_number = 0
render_frame = 0

# Real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

print("Simulation completed.")