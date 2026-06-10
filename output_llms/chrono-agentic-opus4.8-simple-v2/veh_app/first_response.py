import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # chassis spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # no initial rotation
step_size = 1e-3                                                      # integration step (s)
tire_step_size = step_size                                           # tire model step (s)

vis_type_mesh = veh.VisualizationType_MESH                          # detailed mesh visuals
vis_type_prim = veh.VisualizationType_PRIMITIVES                    # primitive-shape visuals

gator = veh.Gator()                                                  # Gator catalog vehicle
gator.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
gator.SetChassisFixed(False)                                         # chassis must be free to move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
gator.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tire on rigid terrain
gator.SetTireStepSize(tire_step_size)                                # tire integration step
gator.Initialize()                                                   # build the vehicle subsystems

gator.SetChassisVisualizationType(vis_type_mesh)                     # chassis as mesh
gator.SetSuspensionVisualizationType(vis_type_prim)                  # suspension as primitives
gator.SetSteeringVisualizationType(vis_type_prim)                    # steering as primitives
gator.SetWheelVisualizationType(vis_type_mesh)                       # wheels as mesh
gator.SetTireVisualizationType(vis_type_mesh)                        # tires as mesh

system = gator.GetSystem()                                            # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())               # report total vehicle mass

terrainLength = 100.0                                                # terrain size in X (m)
terrainWidth = 100.0                                                 # terrain size in Y (m)
terrain = veh.RigidTerrain(system)                                   # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # tire-ground friction
patch_mat.SetRestitution(0.01)                                       # near-inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # ground tint
terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-aware Irrlicht window
vis.SetWindowTitle("Gator Vehicle")                                  # window title
vis.SetWindowSize(1280, 720)                                         # window pixel size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)         # chase cam track point/dist/height
vis.Initialize()                                                     # build the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                      # sky backdrop
vis.AddLightDirectional()                                           # vehicle scenes use directional light
vis.AttachVehicle(gator.GetVehicle())                               # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive keyboard driver
steering_time = 1.0                                                  # s to ramp steering 0->1
throttle_time = 1.0                                                  # s to ramp throttle 0->1
braking_time = 0.3                                                   # s to ramp braking 0->1
render_step_size = 1.0 / 50.0                                        # render cadence (s)
driver.SetSteeringDelta(render_step_size / steering_time)           # steering increment per render step
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle increment per render step
driver.SetBrakingDelta(render_step_size / braking_time)             # braking increment per render step
driver.Initialize()                                                 # finalize the driver

manager = sens.ChSensorManager(system)                              # oversees all sensors
intensity = 1.0                                                     # point-light intensity
manager.scene.AddPointLight(                                        # key fill light
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                        # second fill light
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

offset_pose = chrono.ChFramed(                                      # camera mount on the chassis
    chrono.ChVector3d(-5.0, 0, 1.5),                               # behind & above the chassis origin
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),     # slight downward tilt
)
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),                                         # ride on the vehicle chassis
    30,                                                            # update rate (Hz)
    offset_pose,                                                   # mount frame on the chassis
    1280, 720,                                                     # image resolution
    1.408,                                                         # horizontal FOV (rad)
)
cam.SetName("Chassis Camera")                                      # sensor name
cam.SetLag(0)                                                      # no lag
cam.SetCollectionWindow(0)                                         # instantaneous exposure
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))   # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                        # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))              # save RGB frames
manager.AddSensor(cam)                                             # register the camera

sim_end = 20.0                                                     # simulation duration (s)
render_steps = math.ceil(render_step_size / step_size)            # physics steps per rendered frame
render_every = render_steps                                       # untagged render cadence
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # physics step counter

while vis.Run():
    time = system.GetChTime()                                     # current sim time

    if step_number % render_steps == 0:                          # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                           # current driver command

    driver.Synchronize(time)                                     # update driver
    terrain.Synchronize(time)                                    # update terrain
    gator.Synchronize(time, driver_inputs, terrain)             # update vehicle
    vis.Synchronize(time, driver_inputs)                        # update visualization

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    gator.Advance(step_size)                                     # advance vehicle (steps the system)
    vis.Advance(step_size)                                       # advance visualization
    manager.Update()                                            # update all sensors once per step

    buffer = cam.GetMostRecentRGBA8Buffer()                     # latest camera frame
    if buffer.HasData():                                        # only once the sensor has ticked
        print('Buffer received. Resolution: {0}x{1}'.format(buffer.Width, buffer.Height))


    step_number += 1                                            # advance step counter
    realtime_timer.Spin(step_size)                             # spin in place to match wall clock

    if time >= sim_end:
        break
