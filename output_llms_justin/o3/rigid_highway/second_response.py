import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ----------------------------------------------------------------
# Input files location
# ----------------------------------------------------------------
#
# NOTE:
#   Make sure the environment variable CHRONO_DATA_DIR is defined or
#   point PyChrono to the data folder explicitly with
#   chrono.SetChronoDataPath(<absolute_path_to_data_folder>)
# ----------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # keep default
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------------------------------------------
# Initial vehicle configuration
# ----------------------------------------------------------------
initLoc = chrono.ChVector3d(6, -70, 0.5)
initRot = chrono.QuatFromAngleZ(1.57)        # heading (yaw)  = 90 degrees

vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model             = veh.TireModelType_TMEASY

# ----------------------------------------------------------------
# Contact / solver
# ----------------------------------------------------------------
contact_method = chrono.ChContactMethod_NSC     # NSC  (= penalty-based)
contact_vis    = False

# ----------------------------------------------------------------
# Integrator step sizes
# ----------------------------------------------------------------
step_size       = 1e-3
tire_step_size  = step_size
render_fps      = 50
render_step     = 1.0 / render_fps

# ----------------------------------------------------------------
# Create the HMMWV vehicle
# ----------------------------------------------------------------
vehicle = veh.HMMWV_Full()
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

# Use the Bullet narrow-phase (needed for mesh–mesh contact)
vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystem.Type_BULLET
)

# ----------------------------------------------------------------
# Create rigid-terrain patches
# ----------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

# 1. Highway patch (existing one) -------------------------------------------------
patch_highway = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),
    True,                             # mesh is OBJ
    0.01,                             # sweep sphere radius
    False                             # no automatic texture
)

# Give it a visual mesh
vis_mesh = chrono.ChTriangleMeshConnected()
vis_mesh.LoadWavefrontMesh(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"),
    True, True
)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch_highway.GetGroundBody().AddVisualShape(tri_mesh_shape)

# 2. NEW bump patch ----------------------------------------------------------------
#    (added to satisfy task requirement)
patch_bump = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/bump.obj'),
    True,                              # OBJ mesh
    0.01,                              # sweep sphere radius
    False
)
# colour and texture
patch_bump.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch_bump.SetTexture(
    chrono.GetChronoDataFile('vehicle/terrain/textures/dirt.jpg'),
    6.0, 6.0
)

terrain.Initialize()

# ----------------------------------------------------------------
# Irrlicht visualization
# ----------------------------------------------------------------
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ----------------------------------------------------------------
# Interactive driver (keyboard)
# ----------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0     # from 0 to +/-1 in 1 s
throttle_time = 1.0     # from 0 to 1  in 1 s
braking_time  = 0.3     # from 0 to 1  in 0.3 s
driver.SetSteeringDelta(render_step / steering_time)
driver.SetThrottleDelta(render_step / throttle_time)
driver.SetBrakingDelta(render_step  / braking_time)
driver.Initialize()

# ----------------------------------------------------------------
# Simulation loop
# ----------------------------------------------------------------
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps   = math.ceil(render_step / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
render_frame   = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Module synchronisation
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance state
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment counters
    step_number += 1
    realtime_timer.Spin(step_size)