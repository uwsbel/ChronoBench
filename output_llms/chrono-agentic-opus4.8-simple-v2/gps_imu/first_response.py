import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Data paths for the bundled Chrono + vehicle assets
chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

# Simulation parameters
step_size = 1e-3                                                       # physics step (s)
tire_step_size = 1e-3                                                  # tire substep (s)
render_step_size = 1.0 / 50.0                                          # 50 FPS render cadence
sim_end = 20.0                                                         # stop time (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                                # chassis-origin spawn
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # identity orientation

# --- HMMWV full vehicle on rigid terrain ---
hmmwv = veh.HMMWV_Full()                                               # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                          # MANDATORY — chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                                # tire integration substep
hmmwv.Initialize()                                                   # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)       # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)         # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)          # tire mesh

system = hmmwv.GetSystem()                                            # wrapper owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                # report total vehicle mass

# --- Rigid terrain patch ---
terrain = veh.RigidTerrain(system)                                   # rigid-ground terrain
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # tire-ground friction
patch_mat.SetRestitution(0.01)                                       # near-zero bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)   # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # terrain tint
terrain.Initialize()                                                 # build terrain

# --- Vehicle Irrlicht visualization ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht vis
vis.SetWindowTitle("HMMWV with GPS and IMU sensors")                 # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)         # chase-cam track/dist/height
vis.Initialize()                                                     # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                      # sky background
vis.AddLightDirectional()                                            # directional scene light
vis.AttachVehicle(hmmwv.GetVehicle())                               # bind chassis/wheel visuals

# --- Interactive driver (keyboard) ---
driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to vis
steering_time = 1.0                                                  # s for 0 -> +1 steering
throttle_time = 1.0                                                  # s for 0 -> +1 throttle
braking_time = 0.3                                                   # s for 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)           # per-render steering increment
driver.SetThrottleDelta(render_step_size / throttle_time)           # per-render throttle increment
driver.SetBrakingDelta(render_step_size / braking_time)             # per-render brake increment
driver.Initialize()                                                 # arm the driver

# --- Sensor manager + GPS and IMU on the chassis ---
manager = sens.ChSensorManager(system)                              # oversees all sensors
chassis_body = hmmwv.GetChassisBody()                               # body the sensors ride on

# Sensor mounting pose on the chassis frame
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),                                     # offset from chassis origin
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),         # no rotation
)

# IMU / accelerometer — 4-arg ctor, ChNoiseNone object, own access filter
imu = sens.ChAccelerometerSensor(chassis_body, 100, offset_pose, sens.ChNoiseNone())
imu.SetName("IMU Sensor")                                           # sensor name
imu.SetLag(0)                                                       # no acquisition lag
imu.SetCollectionWindow(0)                                          # instantaneous collection
imu.PushFilter(sens.ChFilterAccelAccess())                         # host access to accel buffer
manager.AddSensor(imu)                                              # register IMU

# GPS — 4th arg is the (lat, lon, alt) reference origin
gps = sens.ChGPSSensor(
    chassis_body, 100, offset_pose,
    chrono.ChVector3d(-89.400, 43.070, 260.0),                      # reference lat/lon/alt
    sens.ChNoiseNone(),                                             # no GPS noise
)
gps.SetName("GPS Sensor")                                           # sensor name
gps.SetLag(0)                                                       # no acquisition lag
gps.SetCollectionWindow(0)                                          # instantaneous collection
gps.PushFilter(sens.ChFilterGPSAccess())                           # host access to GPS buffer
manager.AddSensor(gps)                                              # register GPS

# --- Simulation loop (real-time, render-throttled) ---
render_steps = math.ceil(render_step_size / step_size)              # physics steps per render
realtime_timer = chrono.ChRealtimeStepTimer()                       # spin to wall-clock
step_number = 0                                                     # step counter

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver command

    driver.Synchronize(time)                                       # sync driver
    terrain.Synchronize(time)                                      # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)               # sync vehicle with inputs
    vis.Synchronize(time, driver_inputs)                          # sync vis (HUD)

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    hmmwv.Advance(step_size)                                       # advances the wrapper system
    vis.Advance(step_size)                                         # advance vis

    manager.Update()                                              # pump sensors once per step

    # Read the latest IMU / GPS buffers (truth-style sensor data output)
    acc_buf = imu.GetMostRecentAccelBuffer()                      # most recent accelerometer data
    if acc_buf.HasData():                                          # only after first IMU tick
        acc = acc_buf.GetAccelData()                              # [ax, ay, az] in chassis frame
    gps_buf = gps.GetMostRecentGPSBuffer()                        # most recent GPS data
    if gps_buf.HasData():                                          # only after first GPS tick
        pos = gps_buf.GetGPSData()                                # [lat, lon, alt] of the chassis

    step_number += 1                                              # next step
    realtime_timer.Spin(step_size)                               # match wall-clock to sim time
