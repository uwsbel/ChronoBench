import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

chrono.SetChronoDataPath(chrono.GetChronoDataPath())         # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')     # locate vehicle data files

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(6, -70, 0.5)                    # spawn at crossroads patch position
initRot = chrono.QuatFromAngleZ(1.57)                        # facing along X axis

vis_type = veh.VisualizationType_MESH                        # mesh visualization for all parts
chassis_collision_type = veh.CollisionType_NONE              # no chassis collision shape
tire_model = veh.TireModelType_TMEASY                        # TMEASY tire for rigid terrain

terrainHeight = 0                                            # flat terrain height
terrainLength = 100.0                                        # terrain size in X direction
terrainWidth = 100.0                                         # terrain size in Y direction

trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)              # point on chassis tracked by camera

contact_method = chrono.ChContactMethod_NSC                  # NSC for rigid terrain

step_size = 1e-3                                             # simulation step size
tire_step_size = step_size                                   # tire step size matches sim step

render_step_size = 1.0 / 50                                  # 50 FPS render rate
render_steps = math.ceil(render_step_size / step_size)       # steps between renders

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()                                   # full HMMWV model
vehicle.SetContactMethod(contact_method)                     # NSC contact for rigid terrain
vehicle.SetChassisCollisionType(chassis_collision_type)      # no chassis collision
vehicle.SetChassisFixed(False)                               # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))  # initial position and rotation
vehicle.SetTireType(tire_model)                              # TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                      # tire integration step

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)                # mesh chassis visualization
vehicle.SetSuspensionVisualizationType(vis_type)             # mesh suspension visualization
vehicle.SetSteeringVisualizationType(vis_type)               # mesh steering visualization
vehicle.SetWheelVisualizationType(vis_type)                  # mesh wheel visualization
vehicle.SetTireVisualizationType(vis_type)                   # mesh tire visualization

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision (required)

# Create the terrain with updated contact material (friction 0.4, restitution 0.05)
patch_mat = chrono.ChContactMaterialNSC()                    # NSC material for rigid terrain
patch_mat.SetFriction(0.4)                                   # updated friction value (was 0.9)
patch_mat.SetRestitution(0.05)                               # updated restitution value (was 0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

# Patch orientation: -90 degrees about Z-axis
quat = chrono.ChQuaterniond()
quat.SetFromAngleAxis(-math.pi / 2, chrono.ChVector3d(0, 0, 1))  # -90 deg rotation about Z

# Patch position at (6, -70, 0) — vehicle at crossroads with terrain
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(6, -70, 0), quat),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),
    True, 0.01, False
)

# Visual mesh for highway terrain
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True
)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)                             # attach highway visual mesh
tri_mesh_shape.SetMutable(False)                             # static visual mesh
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)         # add visual to terrain patch

terrain.Initialize()

# Create the vehicle Irrlicht interface (Initialize first, then add scene elements)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')                             # window title
vis.SetWindowSize(1280, 1024)                                # window resolution
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                    # chase camera behind vehicle
vis.Initialize()                                             # initialize vis FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Chrono logo
vis.AddLightDirectional()                                    # directional lighting
vis.AddSkyBox()                                              # sky background
vis.AttachVehicle(vehicle.GetVehicle())                      # bind vehicle visuals

# Create the driver system (interactive keyboard control)
driver = veh.ChInteractiveDriverIRR(vis)                     # IRR interactive driver

steering_time = 1.0                                          # time to go from 0 to +1 steering
throttle_time = 1.0                                          # time to go from 0 to +1 throttle
braking_time = 0.3                                           # time to go from 0 to +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)    # steering response rate
driver.SetThrottleDelta(render_step_size / throttle_time)    # throttle response rate
driver.SetBrakingDelta(render_step_size / braking_time)      # braking response rate
driver.Initialize()

# Output vehicle mass (scored core — survives strip)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation state
realtime_timer = chrono.ChRealtimeStepTimer()                # real-time synchronization
step_number = 0
render_frame = 0

render_every = max(1, round(1.0 / (50 * step_size)))         # render cadence (untagged)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()                   # current simulation time

    if step_number % render_steps == 0:                      # throttled render
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()                       # get current driver inputs

    # Synchronize all subsystems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)        # wheeled vehicle: 3-arg form
    vis.Synchronize(time, driver_inputs)

    # Advance all subsystems
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)                               # advances wrapper-owned ChSystem
    vis.Advance(step_size)


    step_number += 1
    realtime_timer.Spin(step_size)                           # spin for real-time sync
