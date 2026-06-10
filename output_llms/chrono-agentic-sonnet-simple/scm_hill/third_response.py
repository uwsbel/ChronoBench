import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

# vehicle init position and orientation
initLoc = chrono.ChVector3d(-15, 0, 1.2)                            # start off-center so wheels rest on flat terrain
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # no rotation

vis_type = veh.VisualizationType_MESH                                # mesh visualization for vehicle parts
chassis_collision_type = veh.CollisionType_NONE                      # no chassis collision geometry

tire_model = veh.TireModelType_RIGID                                 # rigid tires on rigid terrain
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)                      # camera track point on chassis

contact_method = chrono.ChContactMethod_NSC                          # NSC contact method (changed from SMC)

step_size = 1e-3                                                      # physics step size (s)
tire_step_size = step_size
render_step_size = 1.0 / 20                                          # render at 20 fps
sim_end = 20.0                                                        # simulation duration (s)
render_fps = 50.0                                                     # video capture fps

# create HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)                             # NSC contact method
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)                                      # RIGID tires for rigid terrain
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required after Initialize

# rigid terrain with a single heightmap patch (changed from SCM deformable terrain)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()                            # NSC material to match contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
# single patch with height map and specified texture
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),   # patch frame at origin
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),               # height map for hill terrain
    40.0, 40.0,                                                      # 40 x 40 m patch
    -1.0, 1.0,                                                       # height range -1 to 1 m
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # terrain texture with tiling

terrain.Initialize()

# Irrlicht vehicle visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                             # chase camera
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()                                            # directional light for vehicle demos
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# interactive driver
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0                                                  # time to reach max steering
throttle_time = 1.0                                                  # time to reach max throttle
braking_time = 0.3                                                   # time to reach max braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

render_steps = math.ceil(render_step_size / step_size)              # steps between renders
render_every = max(1, round(1.0 / (render_fps * step_size)))        # video frame cadence (untagged)


realtime_timer = chrono.ChRealtimeStepTimer()                        # real-time synchronisation
step_number = 0

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_every == 0:                              # throttled rendering for video
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)                                       # advances the owned ChSystem
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                                   # real-time pacing
