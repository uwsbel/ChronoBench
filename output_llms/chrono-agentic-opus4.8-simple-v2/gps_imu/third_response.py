import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

# --- simulation parameters (inline locals near use) ---
step_size = 1e-3                                                       # physics step (s)
tire_step_size = step_size                                             # tire integration step
sim_end = 20.0                                                         # total simulated time (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                               # chassis spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # chassis spawn orientation

# --- HMMWV vehicle (full model on rigid terrain) ---
hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                         # MANDATORY — chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                               # tire sub-step
hmmwv.Initialize()                                                    # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # tire mesh

system = hmmwv.GetSystem()                                            # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED — contact scene
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

# --- rigid terrain (flat patch) ---
terrain = veh.RigidTerrain(system)                                   # rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                           # tire-ground friction
patch_mat.SetRestitution(0.01)                                       # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # patch tint
terrain.Initialize()                                                  # build the terrain

# --- Irrlicht visualization (vehicle-specific system) ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle Irrlicht vis
vis.SetWindowTitle("HMMWV GPS/IMU Sensors")                          # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)        # chase cam track point/dist/height
vis.Initialize()                                                     # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # directional light (vehicle truths)
vis.AttachVehicle(hmmwv.GetVehicle())                               # bind vehicle visuals

# --- interactive driver with a scripted constant maneuver ---
driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to vis
steering_time = 1.0                                                  # s to reach full steering
throttle_time = 1.0                                                  # s to reach full throttle
braking_time = 0.3                                                   # s to reach full braking
driver.SetSteeringDelta(step_size / steering_time)                  # steering ramp rate
driver.SetThrottleDelta(step_size / throttle_time)                  # throttle ramp rate
driver.SetBrakingDelta(step_size / braking_time)                    # braking ramp rate
driver.Initialize()                                                 # build the driver

# --- sensor manager + IMU and GPS attached to the chassis ---
manager = sens.ChSensorManager(system)                              # oversee all sensors
chassis_body = hmmwv.GetChassisBody()                               # body the sensors ride on

# IMU (accelerometer) — offset pose moved to (0, 0, 1)
imu_offset_pose = chrono.ChFramed(                                   # IMU mount frame on the chassis
    chrono.ChVector3d(0, 0, 1),                                     # offset (0, 0, 1)
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),        # no rotation
)
imu_update_rate = 100.0                                              # IMU sample rate (Hz)
imu_noise_model = sens.ChNoiseNone()                                # no IMU noise
imu = sens.ChAccelerometerSensor(chassis_body, imu_update_rate, imu_offset_pose, imu_noise_model)
imu.SetName("IMU Sensor")                                            # sensor name
imu.SetLag(0)                                                        # no lag
imu.SetCollectionWindow(0)                                           # instantaneous collection
imu.PushFilter(sens.ChFilterAccelAccess())                          # host access to accel data
manager.AddSensor(imu)                                               # register IMU

# GPS — reference origin (lat, lon, alt); offset pose on the chassis
gps_offset_pose = chrono.ChFramed(                                   # GPS mount frame on the chassis
    chrono.ChVector3d(0, 0, 1),                                     # offset (0, 0, 1)
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),        # no rotation
)
gps_update_rate = 10.0                                               # GPS sample rate (Hz)
gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)          # lat/lon/alt reference origin
gps_noise_model = sens.ChNoiseNone()                                # no GPS noise
gps = sens.ChGPSSensor(chassis_body, gps_update_rate, gps_offset_pose, gps_reference, gps_noise_model)
gps.SetName("GPS Sensor")                                           # sensor name
gps.SetLag(0)                                                       # no lag
gps.SetCollectionWindow(0)                                          # instantaneous collection
gps.PushFilter(sens.ChFilterGPSAccess())                           # host access to GPS data
manager.AddSensor(gps)                                              # register GPS

# --- render cadence + recording scaffolding ---
render_step_size = 1.0 / 50.0                                       # render once per 1/50 s
render_every = max(1, round(render_step_size / step_size))         # untagged cadence constant

# --- main loop: scripted constant steering 0.6 / throttle 0.5 ---
while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()                                               # begin frame
    vis.Render()                                                   # draw scene
    vis.EndScene()                                                 # end frame
    for _ in range(render_every):
        sim_time = hmmwv.GetSystem().GetChTime()                   # current sim time

        driver_inputs = driver.GetInputs()                         # base driver inputs
        driver_inputs.m_steering = 0.6                             # constant steering 0.6
        driver_inputs.m_throttle = 0.5                             # constant throttle 0.5
        driver_inputs.m_braking = 0.0                              # no braking

        driver.Synchronize(sim_time)                               # sync driver
        terrain.Synchronize(sim_time)                              # sync terrain
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)       # sync vehicle with inputs
        vis.Synchronize(sim_time, driver_inputs)                  # sync vis HUD

        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        hmmwv.Advance(step_size)                                   # advance vehicle (steps system)
        vis.Advance(step_size)                                     # advance vis

        manager.Update()                                           # pump sensors once per step

        imu_buffer = imu.GetMostRecentAccelBuffer()               # read IMU buffer
        if imu_buffer.HasData():                                   # only after first IMU tick
            accel = imu_buffer.GetAccelData()                     # latest accel sample [x, y, z]

        gps_buffer = gps.GetMostRecentGPSBuffer()                 # read GPS buffer
        if gps_buffer.HasData():                                  # only after first GPS tick
            gps_data = gps_buffer.GetGPSData()                    # latest GPS sample [lon, lat, alt]

        if hmmwv.GetSystem().GetChTime() >= sim_end:
            break
