import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                        # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                    # locate vehicle data files

initLoc = chrono.ChVector3d(6, -70, 0.5)                                    # initial vehicle location on highway
initRot = chrono.QuatFromAngleZ(1.57)                                       # rotated 90 deg to face highway direction

vis_type = veh.VisualizationType_MESH                                        # mesh visualization for all vehicle parts
chassis_collision_type = veh.CollisionType_NONE                              # no chassis collision geometry
tire_model = veh.TireModelType_TMEASY                                        # TMEASY tire model as requested

trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)                             # chase-camera track point on chassis

contact_method = chrono.ChContactMethod_NSC                                  # NSC contact for rigid terrain
step_size = 1e-3                                                             # simulation step size (1 ms)
tire_step_size = step_size                                                   # tire step matches simulation step
render_step_size = 1.0 / 50                                                  # 50 FPS render cadence
render_steps = math.ceil(render_step_size / step_size)                      # physics steps per render frame

vehicle = veh.HMMWV_Full()                                                   # full HMMWV model
vehicle.SetContactMethod(contact_method)                                     # NSC for rigid terrain
vehicle.SetChassisCollisionType(chassis_collision_type)                      # no chassis primitive collision
vehicle.SetChassisFixed(False)                                               # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))               # spawn on highway
vehicle.SetTireType(tire_model)                                              # TMEASY as requested
vehicle.SetTireStepSize(tire_step_size)                                      # tire integration step

vehicle.Initialize()                                                         # build vehicle bodies + joints

vehicle.SetChassisVisualizationType(vis_type)                                # mesh chassis visual
vehicle.SetSuspensionVisualizationType(vis_type)                             # mesh suspension visual
vehicle.SetSteeringVisualizationType(vis_type)                               # mesh steering visual
vehicle.SetWheelVisualizationType(vis_type)                                  # mesh wheel visual
vehicle.SetTireVisualizationType(vis_type)                                   # mesh tire visual

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())                      # report total vehicle mass

# Custom mesh terrain — Highway_col.obj for collision, Highway_vis.obj for visuals
patch_mat = chrono.ChContactMaterialNSC()                                    # NSC contact material
patch_mat.SetFriction(0.9)                                                   # highway friction coefficient
patch_mat.SetRestitution(0.01)                                               # low restitution
terrain = veh.RigidTerrain(vehicle.GetSystem())                              # rigid terrain on vehicle system
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),           # origin, no rotation
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),      # collision mesh
    True, 0.01, False)                                                       # use convex decomp, contact offset, no mesh vis
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)           # load highway visual mesh
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()                         # visual shape wrapper
tri_mesh_shape.SetMesh(vis_mesh)                                             # attach mesh
tri_mesh_shape.SetMutable(False)                                             # static mesh — no deformation
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)                         # attach vis mesh to terrain body
terrain.Initialize()                                                         # finalize terrain

# Vehicle Irrlicht visualization — ChWheeledVehicleVisualSystemIrrlicht for vehicle scenes
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Highway Demo')                                     # window title
vis.SetWindowSize(1280, 1024)                                                # window dimensions
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                                     # chase camera behind chassis
vis.Initialize()                                                             # create Irrlicht device (FIRST)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))             # Chrono logo
vis.AddLightDirectional()                                                    # directional light for vehicle scenes
vis.AddSkyBox()                                                              # sky box backdrop
vis.AttachVehicle(vehicle.GetVehicle())                                      # bind vehicle meshes to scene

# Interactive driver — keyboard steering/throttle/braking
driver = veh.ChInteractiveDriverIRR(vis)                                     # IRR interactive driver (vis required)
steering_time = 1.0                                                          # seconds to reach max steering
throttle_time = 1.0                                                          # seconds to reach max throttle
braking_time = 0.3                                                           # seconds to reach max braking
driver.SetSteeringDelta(render_step_size / steering_time)                    # steering rate per render step
driver.SetThrottleDelta(render_step_size / throttle_time)                    # throttle rate per render step
driver.SetBrakingDelta(render_step_size / braking_time)                      # braking rate per render step
driver.Initialize()                                                          # finalize driver

# Simulation loop — real-time at 50 FPS with render throttling
realtime_timer = chrono.ChRealtimeStepTimer()                                # real-time pacing
step_number = 0                                                              # physics step counter


while vis.Run():
    time = vehicle.GetSystem().GetChTime()                                   # current simulation time

    if step_number % render_steps == 0:                                      # throttled render at 50 FPS
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                                       # get current driver inputs

    driver.Synchronize(time)                                                 # update driver state
    terrain.Synchronize(time)                                                # update terrain state
    vehicle.Synchronize(time, driver_inputs, terrain)                        # update vehicle with inputs + terrain
    vis.Synchronize(time, driver_inputs)                                     # update visual system


    driver.Advance(step_size)                                                # advance driver
    terrain.Advance(step_size)                                               # advance terrain
    vehicle.Advance(step_size)                                               # advance vehicle + system
    vis.Advance(step_size)                                                   # advance visual system

    step_number += 1
    realtime_timer.Spin(step_size)                                           # pace to real time
