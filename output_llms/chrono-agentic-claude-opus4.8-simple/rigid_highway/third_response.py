import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core Chrono data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data path

initLoc = chrono.ChVector3d(6, -70, 0.5)                             # spawn on the highway mesh
initRot = chrono.QuatFromAngleZ(1.57)                                # face +Y along the road
contact_method = chrono.ChContactMethod_NSC                          # rigid-terrain default
chassis_collision_type = veh.CollisionType_NONE                      # no chassis collision shape

step_size = 1e-3                                                     # dynamics step
tire_step_size = step_size                                           # tire force step
render_step_size = 1.0 / 50.0                                        # 50 fps rendering

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog wrapper
hmmwv.SetContactMethod(contact_method)                               # NSC contacts
hmmwv.SetChassisCollisionType(chassis_collision_type)               # disable chassis collision
hmmwv.SetChassisFixed(False)                                         # chassis free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))         # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                                # tire integration step
hmmwv.Initialize()                                                   # build the vehicle

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # diagnostic banner

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)     # mesh suspension
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)       # mesh steering
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # mesh wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # mesh tires

terrain = veh.RigidTerrain(hmmwv.GetSystem())                        # rigid terrain on shared system

patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.4)                                           # updated friction
patch_mat.SetRestitution(0.05)                                       # updated restitution

patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         veh.GetDataFile("terrain/meshes/Highway_col.obj"),  # collision mesh
                         True, 0.01, False)                          # static, thickness, no viz here

vis_mesh = chrono.ChTriangleMeshConnected()                          # visual mesh container
vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), False, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()                       # visual mesh shape
vis_shape.SetMesh(vis_mesh)                                          # attach mesh
vis_shape.SetMutable(False)                                          # static visual
patch.GetGroundBody().AddVisualShape(vis_shape)                      # render the highway

bump_rot = chrono.QuatFromAngleZ(-math.pi / 2.0)                     # -90 deg about Z
bump_patch = terrain.AddPatch(patch_mat,
                              chrono.ChCoordsysd(chrono.ChVector3d(6, -70, 0), bump_rot),  # cross-roads pose
                              veh.GetDataFile("terrain/meshes/bump.obj"))  # bump patch mesh
bump_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                  # bump patch color
bump_patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture

terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle Irrlicht visual system
vis.SetWindowTitle('HMMWV on Highway Mesh')                          # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(-3.0, 0.0, 1.1), 6.0, 0.5)     # chase camera
vis.Initialize()                                                     # init first (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))    # logo
vis.AddLightDirectional()                                            # directional light
vis.AddSkyBox()                                                      # sky box
vis.AttachVehicle(hmmwv.GetVehicle())                                # attach vehicle to vis

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive keyboard driver
steering_time = 1.0                                                  # s to full steering
throttle_time = 1.0                                                  # s to full throttle
braking_time = 0.3                                                   # s to full brake
driver.SetSteeringDelta(render_step_size / steering_time)           # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)             # braking rate
driver.Initialize()                                                  # init driver

render_steps = math.ceil(render_step_size / step_size)              # physics steps per frame
realtime_timer = chrono.ChRealtimeStepTimer()                       # real-time pacing
step_number = 0                                                     # step counter


while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                            # current sim time

    if step_number % render_steps == 0:                            # throttled render
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver inputs

    driver.Synchronize(time)                                       # sync driver
    terrain.Synchronize(time)                                      # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)                # sync vehicle
    vis.Synchronize(time, driver_inputs)                           # sync visuals

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    hmmwv.Advance(step_size)                                       # advance vehicle + system
    vis.Advance(step_size)                                         # advance visuals


    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # real-time pacing
