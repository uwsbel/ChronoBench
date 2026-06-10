import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

time_step = 1e-3                                                       # integration step (s)
tire_step = 1e-3                                                       # tire substep (s)
sim_end = 10.0                                                         # simulation end time (s)

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # chassis spawn (m), above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity heading

hmmwv = veh.HMMWV_Full()                                              # catalog HMMWV wrapper (owns its system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision body
hmmwv.SetChassisFixed(False)                                         # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step)                                     # tire integration substep
hmmwv.Initialize()                                                   # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)    # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)      # mesh wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)       # mesh tires

system = hmmwv.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # flat rigid ground under the vehicle
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                          # ground friction
patch_mat.SetRestitution(0.01)                                      # nearly no bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200 x 200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # sandy color
terrain.Initialize()                                                # build the terrain collision/visual

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-aware Irrlicht window
vis.SetWindowTitle("HMMWV Depth Camera")                            # window title
vis.SetWindowSize(1280, 1024)                                       # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase cam track point, dist, height
vis.Initialize()                                                    # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # vehicle demos use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                              # bind the vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                           # real-time interactive driver (truth default)
steering_time = 1.0                                                # s to ramp steering 0 -> 1
throttle_time = 1.0                                                # s to ramp throttle 0 -> 1
braking_time = 0.3                                                 # s to ramp brake 0 -> 1
render_step_size = 1.0 / 50.0                                      # render cadence (s)
driver.SetSteeringDelta(render_step_size / steering_time)         # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)         # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)           # brake rate
driver.Initialize()                                               # arm the driver

manager = sens.ChSensorManager(system)                            # sensor manager on the vehicle system
intensity = 1.0                                                   # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)   # point light
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)   # second point light
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4),
                           chrono.ChColor(intensity, intensity, intensity), 500.0,
                           chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))   # area light

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2),                                # offset on the chassis (m), in body frame
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),   # slight downward tilt about +Y
)
depth_cam = sens.ChDepthCamera(
    hmmwv.GetChassisBody(),                                       # ride on the chassis
    10,                                                          # update rate (Hz) — physical rate
    offset_pose,                                                 # offset pose on the body
    1280,                                                       # image width (px)
    720,                                                        # image height (px)
    1.408,                                                      # horizontal FOV (rad)
)
depth_cam.SetName("Depth Camera")                                # sensor name
depth_cam.SetLag(0)                                              # no lag
depth_cam.SetCollectionWindow(0)                                # instantaneous collection
depth_cam.SetMaxDepth(30)                                        # clip depth at 30 m
depth_cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Depth Map"))  # live depth-map preview
manager.AddSensor(depth_cam)                                    # register the depth camera

render_every = max(1, round(render_step_size / time_step))     # untagged render-cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                  # keep wall-clock in sync with sim time
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                           # begin the Irrlicht frame
    vis.Render()                                               # draw the scene
    vis.EndScene()                                             # end the Irrlicht frame
    for _ in range(render_every):
        sim_time = system.GetChTime()                          # current sim time
        driver_inputs = driver.GetInputs()                     # latest driver inputs
        driver.Synchronize(sim_time)                           # sync the driver
        terrain.Synchronize(sim_time)                          # sync the terrain
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)    # sync the vehicle with terrain
        vis.Synchronize(sim_time, driver_inputs)               # sync the visual/HUD
        driver.Advance(time_step)                              # advance the driver
        terrain.Advance(time_step)                             # advance the terrain
        hmmwv.Advance(time_step)                               # advance the vehicle (steps the system)
        vis.Advance(time_step)                                 # advance the visual
        manager.Update()                                       # pump the depth camera once per physics step

        pos = hmmwv.GetVehicle().GetPos()                      # chassis world position
        heading = hmmwv.GetVehicle().GetRot().GetCardanAnglesZYX().z  # heading (yaw, rad)
        print("Time: ", sim_time, " Pos: ", pos.x, pos.y, pos.z, " Heading: ", heading)  # log vehicle state

        realtime_timer.Spin(time_step)                         # spin to real time
        if system.GetChTime() >= sim_end:
            break
