import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

# -----------------------------------------------------------------------------
# Initialize the PyChrono environment and set the data paths
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions
terrainHeight = 0
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# -----------------------------------------------------------------------------
# Create the HMMWV vehicle, set parameters, and initialize
# -----------------------------------------------------------------------------
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

# Set visualization types for vehicle subsystems
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# Output vehicle mass
print("Vehicle mass: ", hmmwv.GetVehicle().GetMass())

# -----------------------------------------------------------------------------
# Create the terrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(hmmwv.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -----------------------------------------------------------------------------
# Create the vehicle Irrlicht interface
# -----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV with IMU and GPS Sensors')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# -----------------------------------------------------------------------------
# Create the interactive driver system
# -----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0   # time to go from 0 to +1 (or 0 to -1)
throttle_time = 1.0   # time to go from 0 to +1
braking_time = 0.3    # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# -----------------------------------------------------------------------------
# Create the sensor manager
# -----------------------------------------------------------------------------
manager = sens.ChSensorManager(hmmwv.GetSystem())

# Get the chassis body to attach sensors to
chassis_body = hmmwv.GetChassisBody()

# ---------------------- IMU (Accelerometer + Gyroscope + Magnetometer) -------
# Sensor update rate (Hz)
imu_update_rate = 100.0
# Offset pose of the IMU relative to the chassis reference frame
imu_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                  chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0)))
# Lag and collection window
imu_lag = 0.0
imu_collection_time = 0.0

# Noise models (none for simplicity)
acc_noise_none = sens.ChNoiseNone()
gyro_noise_none = sens.ChNoiseNone()
mag_noise_none = sens.ChNoiseNone()

# Accelerometer
acc = sens.ChAccelerometerSensor(chassis_body,
                                 imu_update_rate,
                                 imu_offset_pose,
                                 acc_noise_none)
acc.SetName("Accelerometer")
acc.SetLag(imu_lag)
acc.SetCollectionWindow(imu_collection_time)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# Gyroscope
gyro = sens.ChGyroscopeSensor(chassis_body,
                              imu_update_rate,
                              imu_offset_pose,
                              gyro_noise_none)
gyro.SetName("Gyroscope")
gyro.SetLag(imu_lag)
gyro.SetCollectionWindow(imu_collection_time)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# Magnetometer
gps_reference = chrono.ChVector3d(-121.75, 38.55, 0.0)  # lon, lat, altitude
mag = sens.ChMagnetometerSensor(chassis_body,
                                imu_update_rate,
                                imu_offset_pose,
                                mag_noise_none,
                                gps_reference)
mag.SetName("Magnetometer")
mag.SetLag(imu_lag)
mag.SetCollectionWindow(imu_collection_time)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# ---------------------- GPS --------------------------------------------------
gps_update_rate = 10.0
gps_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                  chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0)))
gps_lag = 0.0
gps_collection_time = 0.0
gps_noise_none = sens.ChNoiseNone()

gps = sens.ChGPSSensor(chassis_body,
                       gps_update_rate,
                       gps_offset_pose,
                       gps_reference,
                       gps_noise_none)
gps.SetName("GPS")
gps.SetLag(gps_lag)
gps.SetCollectionWindow(gps_collection_time)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
# Number of simulation steps between two render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print("Starting simulation loop...")

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Render scene at the prescribed FPS
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # ---------------------- Update sensor manager ---------------------------
    manager.Update()

    # Retrieve and process the most recent IMU data
    acc_buffer = acc.GetMostRecentAccelBuffer()
    if acc_buffer.HasData():
        acc_data = acc_buffer.GetAccelData()
        print(f"[t={time:6.3f}s] Accelerometer: "
              f"X={acc_data.X:.3f}, Y={acc_data.Y:.3f}, Z={acc_data.Z:.3f}")

    gyro_buffer = gyro.GetMostRecentGyroBuffer()
    if gyro_buffer.HasData():
        gyro_data = gyro_buffer.GetGyroData()
        print(f"[t={time:6.3f}s] Gyroscope:     "
              f"X={gyro_data.X:.3f}, Y={gyro_data.Y:.3f}, Z={gyro_data.Z:.3f}")

    # Retrieve and process the most recent GPS data
    gps_buffer = gps.GetMostRecentGPSBuffer()
    if gps_buffer.HasData():
        gps_data = gps_buffer.GetGPSData()
        print(f"[t={time:6.3f}s] GPS: "
              f"Lat={gps_data[1]:.6f}, Lon={gps_data[0]:.6f}, Alt={gps_data[2]:.3f}")

    # ---------------------- Synchronize all modules -------------------------
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # ---------------------- Advance all modules -----------------------------
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to render simulation correctly
    realtime_timer.Spin(step_size)

print("Simulation finished.")