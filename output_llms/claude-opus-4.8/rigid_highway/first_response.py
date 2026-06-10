import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# ----------------------------------------------------------------------------
# Initialize the PyChrono data path (for vehicle models, textures, etc.)
# ----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------------------------------------------------------
# Simulation parameters
# ----------------------------------------------------------------------------
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Contact method (SMC = smooth/penalty, NSC = non-smooth/complementarity)
contact_method = chrono.ChContactMethod_NSC

# Visualization type for vehicle parts (MESH)
vis_type = veh.VisualizationType_MESH

# Collision type for the chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model: TMEASY
tire_model = veh.TireModelType_TMEASY

# Rendering and simulation step
render_step_size = 1.0 / 50    # FPS = 50
step_size = 1e-3               # Dynamics time step

# Terrain mesh files
terrain_col_mesh = veh.GetDataFile("terrain/meshes/Highway_col.obj")
terrain_vis_mesh = veh.GetDataFile("terrain/meshes/Highway_vis.obj")

# ----------------------------------------------------------------------------
# Create and configure the HMMWV vehicle
# ----------------------------------------------------------------------------
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(step_size)

# Initialize the vehicle
hmmwv.Initialize()

# Set visualization type for each subsystem to MESH
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# ----------------------------------------------------------------------------
# Create the custom mesh terrain
# ----------------------------------------------------------------------------
terrain = veh.RigidTerrain(hmmwv.GetSystem())

# Define a contact material for the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Add a patch using the collision mesh
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrain_col_mesh,
                         True,    # connected mesh (compute contact)
                         0.01,    # sweep sphere radius
                         False)   # do not visualize collision mesh

# Add the visual mesh for the terrain
vis_mesh = chrono.ChTriangleMeshConnected()
vis_mesh.LoadWavefrontMesh(terrain_vis_mesh, True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(vis_mesh)
vis_shape.SetName("Highway_visual")
vis_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(vis_shape)

terrain.Initialize()

# ----------------------------------------------------------------------------
# Create the Irrlicht visualization system
# ----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Custom Highway Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# ----------------------------------------------------------------------------
# Create the interactive driver system (keyboard control)
# ----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Set time response for steering, throttle and braking inputs
steering_time = 1.0   # time to go from 0 to +1 (or 0 to -1)
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ----------------------------------------------------------------------------
# Simulation loop (real time, 50 FPS)
# ----------------------------------------------------------------------------
# Number of simulation steps between two render frames
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

# Real-time timer to keep simulation pacing
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Render scene at the desired FPS
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Collect driver inputs from keyboard
    driver_inputs = driver.GetInputs()

    # Update modules (synchronize)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    # Enforce real-time pacing
    realtime_timer.Spin(step_size)