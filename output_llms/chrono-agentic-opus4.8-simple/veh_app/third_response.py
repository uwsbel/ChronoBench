import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

time_step = 1e-3                                                       # integration step (s)
sim_end = 10.0                                                         # simulation end time (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                               # vehicle spawn location
init_rot = chrono.QUNIT                                               # vehicle spawn orientation

hmmwv = veh.HMMWV_Full()                                               # full HMMWV catalog vehicle
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                          # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tires on rigid road
hmmwv.SetTireStepSize(time_step)                                     # tire integration step
hmmwv.Initialize()                                                    # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)      # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)        # mesh wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)         # mesh tires

system = hmmwv.GetSystem()                                            # wrapper owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                            # tire-ground friction
patch_mat.SetRestitution(0.01)                                        # low bounciness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # 200 x 200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # tint the ground
terrain.Initialize()                                                  # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                      # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV Depth Camera")                              # window title
vis.SetWindowSize(1280, 1024)                                         # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)          # chase-cam track point/dist/height
vis.Initialize()                                                      # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo (after Initialize)
vis.AddSkyBox()                                                       # sky box
vis.AddLightDirectional()                                            # directional light (vehicle truths)
vis.AttachVehicle(hmmwv.GetVehicle())                                # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver bound to vis
render_step_size = 1.0 / 50.0                                        # render cadence (s)
driver.SetSteeringDelta(render_step_size / 1.0)                     # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                    # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                     # braking ramp rate
driver.Initialize()                                                  # init the driver

manager = sens.ChSensorManager(system)                              # sensor manager on the vehicle system
intensity = 1.0
manager.scene.AddPointLight(                                          # point light for the camera render
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

offset_pose = chrono.ChFramed(                                        # depth-camera offset on the chassis
    chrono.ChVector3d(-5.0, 0, 2),                                   # offset pose (-5, 0, 2)
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),        # no tilt
)
depth_cam = sens.ChDepthCamera(                                       # depth camera sensor
    hmmwv.GetChassisBody(),                                          # rides on the chassis
    30,                                                              # update rate (Hz) — physical
    offset_pose,                                                     # offset pose on the body
    1280, 720,                                                       # image width, height
    1.408,                                                           # horizontal FOV (rad)
    30,                                                              # max depth (m)
)
depth_cam.SetName("Depth Camera")                                    # sensor name
depth_cam.SetLag(0)                                                  # no lag
depth_cam.SetCollectionWindow(0)                                     # instantaneous collection
depth_cam.PushFilter(sens.ChFilterDepthAccess())                    # host access to depth buffer
depth_cam.PushFilter(sens.ChFilterDepthToRGBA8())                   # depth -> RGBA8 for visualization
depth_cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Depth Map"))  # live depth-map preview
depth_cam.PushFilter(sens.ChFilterSave("cam/depth/"))               # save depth-map PNGs
manager.AddSensor(depth_cam)                                         # register the sensor

render_steps = math.ceil(render_step_size / time_step)               # physics steps per rendered frame

realtime_timer = chrono.ChRealtimeStepTimer()                        # real-time pacing
step_number = 0                                                      # loop step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver inputs

    driver.Synchronize(time)                                       # sync driver
    terrain.Synchronize(time)                                      # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)               # sync vehicle
    vis.Synchronize(time, driver_inputs)                          # sync visualization

    driver.Advance(time_step)                                     # advance driver
    terrain.Advance(time_step)                                    # advance terrain
    hmmwv.Advance(time_step)                                      # advances the wrapper-owned system
    vis.Advance(time_step)                                        # advance visualization
    manager.Update()                                              # pump the depth camera once per step

    chassis = hmmwv.GetChassisBody()                              # chassis body for state log
    pos = chassis.GetPos()                                        # chassis position (X, Y, Z)
    heading = chassis.GetRot().GetCardanAnglesZYX().z             # heading (yaw) angle
    print("t = ", time, " pos = ", pos.x, pos.y, pos.z, " heading = ", heading)  # log vehicle state every step

    step_number += 1                                              # advance loop counter
    realtime_timer.Spin(time_step)                                # spin to match wall clock
