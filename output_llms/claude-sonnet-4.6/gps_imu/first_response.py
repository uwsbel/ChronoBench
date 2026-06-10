import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math

# =============================================================================
# Initialize PyChrono environment
# =============================================================================

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (NONE, PRIMITIVES, HULLS, MESH)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 200.0
terrainWidth = 200.0

# Point tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# =============================================================================
# Create the HMMWV vehicle
# =============================================================================

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# Print vehicle mass
print("Vehicle mass: ", hmmwv.GetVehicle().GetMass())

# =============================================================================
# Create the terrain
# =============================================================================

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

# =============================================================================
# Create the driver system
# =============================================================================

driver = veh.ChInteractiveDriverIRR(vis)  # Will be replaced after vis creation

# We'll create the interactive driver after creating the visualization
# For now, use a path follower or simple driver
driver = veh.ChDriver(hmmwv.GetVehicle())

# =============================================================================
# Create the Irrlicht visualization
# =============================================================================

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# =============================================================================
# Create the interactive driver (after vis)
# =============================================================================

driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0   # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0   # time to go from 0 to +1
braking_time = 0.3    # time to go from 0 to +1

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# =============================================================================
# Create Sensor Manager and add IMU & GPS sensors
# =============================================================================

manager = sens.ChSensorManager(hmmwv.GetSystem())

# Set up lighting for sensor manager
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100),
                             chrono.ChColor(1, 1, 1), 5000)

# -------------------------------------------------------
# IMU Sensor (Accelerometer + Gyroscope + Magnetometer)
# -------------------------------------------------------

# IMU update rate
imu_update_rate = 100  # Hz

# Offset pose for IMU (attached to chassis)
imu_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

# Accelerometer noise model
acc_noise_none = sens.ChNoiseNone()

# Gyroscope noise model  
gyro_noise_none = sens.ChNoiseNone()

# Magnetometer noise model
mag_noise_none = sens.ChNoiseNone()

# Create accelerometer sensor
acc = sens.ChAccelerometerSensor(
    hmmwv.GetVehicle().GetChassisBody(),  # body to which sensor is attached
    imu_update_rate,                       # update rate in Hz
    imu_offset_pose,                       # offset pose
    acc_noise_none                         # noise model
)
acc.SetName("IMU - Accelerometer")
acc.SetLag(0)
acc.SetCollectionWindow(1.0 / imu_update_rate)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# Create gyroscope sensor
gyro = sens.ChGyroscopeSensor(
    hmmwv.GetVehicle().GetChassisBody(),
    imu_update_rate,
    imu_offset_pose,
    gyro_noise_none
)
gyro.SetName("IMU - Gyroscope")
gyro.SetLag(0)
gyro.SetCollectionWindow(1.0 / imu_update_rate)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# Create magnetometer sensor
mag_loc = chrono.ChVector3d(47.6, -122.1, 0)  # Reference location (lat/lon)
mag = sens.ChMagnetometerSensor(
    hmmwv.GetVehicle().GetChassisBody(),
    imu_update_rate,
    imu_offset_pose,
    mag_noise_none,
    mag_loc
)
mag.SetName("IMU - Magnetometer")
mag.SetLag(0)
mag.SetCollectionWindow(1.0 / imu_update_rate)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# -------------------------------------------------------
# GPS Sensor
# -------------------------------------------------------

# GPS update rate
gps_update_rate = 10  # Hz

# GPS offset pose
gps_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

# GPS reference location (latitude, longitude, altitude)
gps_reference = chrono.ChVector3d(-89.4, 43.07, 260.0)

# GPS noise model
gps_noise_none = sens.ChNoiseNone()

# Create GPS sensor
gps = sens.ChGPSSensor(
    hmmwv.GetVehicle().GetChassisBody(),
    gps_update_rate,
    gps_offset_pose,
    gps_reference,
    gps_noise_none
)
gps.SetName("GPS")
gps.SetLag(0)
gps.SetCollectionWindow(1.0 / gps_update_rate)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# =============================================================================
# Simulation loop
# =============================================================================

# Output simulation parameters
print("============================================================")
print("  HMMWV Vehicle Simulation with IMU and GPS Sensors")
print("============================================================")
print(f"  Vehicle mass:     {hmmwv.GetVehicle().GetMass():.2f} kg")
print(f"  Step size:        {step_size} s")
print(f"  IMU update rate:  {imu_update_rate} Hz")
print(f"  GPS update rate:  {gps_update_rate} Hz")
print("============================================================")

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Sensor data accumulators for output
acc_data = None
gyro_data = None
mag_data = None
gps_data = None

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Render scene at specified frequency
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    # Update sensor manager - process all sensor data
    manager.Update()

    # -------------------------------------------------------
    # Access and process IMU sensor data
    # -------------------------------------------------------
    
    # Access accelerometer data
    acc_buffer = acc.GetMostRecentBuffer()
    if acc_buffer.HasData():
        acc_data = acc_buffer.GetData()
        # acc_data contains [ax, ay, az] in m/s^2
        if step_number % (render_steps * 10) == 0:
            print(f"[t={time:.3f}s] Accelerometer: "
                  f"ax={acc_data[0]:.3f}, "
                  f"ay={acc_data[1]:.3f}, "
                  f"az={acc_data[2]:.3f} m/s^2")

    # Access gyroscope data
    gyro_buffer = gyro.GetMostRecentBuffer()
    if gyro_buffer.HasData():
        gyro_data = gyro_buffer.GetData()
        # gyro_data contains [wx, wy, wz] in rad/s
        if step_number % (render_steps * 10) == 0:
            print(f"[t={time:.3f}s] Gyroscope:     "
                  f"wx={gyro_data[0]:.4f}, "
                  f"wy={gyro_data[1]:.4f}, "
                  f"wz={gyro_data[2]:.4f} rad/s")

    # Access magnetometer data
    mag_buffer = mag.GetMostRecentBuffer()
    if mag_buffer.HasData():
        mag_data = mag_buffer.GetData()
        # mag_data contains [mx, my, mz] in uT (microtesla)
        if step_number % (render_steps * 10) == 0:
            print(f"[t={time:.3f}s] Magnetometer:  "
                  f"mx={mag_data[0]:.4f}, "
                  f"my={mag_data[1]:.4f}, "
                  f"mz={mag_data[2]:.4f} uT")

    # -------------------------------------------------------
    # Access and process GPS sensor data
    # -------------------------------------------------------
    
    gps_buffer = gps.GetMostRecentBuffer()
    if gps_buffer.HasData():
        gps_data = gps_buffer.GetData()
        # gps_data contains [latitude, longitude, altitude]
        if step_number % (render_steps * 10) == 0:
            print(f"[t={time:.3f}s] GPS:           "
                  f"lat={gps_data[0]:.6f}°, "
                  f"lon={gps_data[1]:.6f}°, "
                  f"alt={gps_data[2]:.2f}m")

    # -------------------------------------------------------
    # Vehicle state output at regular intervals
    # -------------------------------------------------------
    if step_number % (render_steps * 20) == 0:
        vehicle_pos = hmmwv.GetVehicle().GetPos()
        vehicle_speed = hmmwv.GetVehicle().GetSpeed()
        print(f"\n--- Vehicle State [t={time:.2f}s] ---")
        print(f"  Position:  ({vehicle_pos.x:.2f}, {vehicle_pos.y:.2f}, {vehicle_pos.z:.2f}) m")
        print(f"  Speed:     {vehicle_speed:.2f} m/s ({vehicle_speed * 3.6:.2f} km/h)")
        print(f"  Mass:      {hmmwv.GetVehicle().GetMass():.2f} kg")
        print(f"  Throttle:  {driver_inputs.m_throttle:.3f}")
        print(f"  Steering:  {driver_inputs.m_steering:.3f}")
        print(f"  Braking:   {driver_inputs.m_braking:.3f}")

    # Increment step number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

print("\nSimulation completed.")
print(f"Total simulation steps: {step_number}")
print(f"Total render frames:    {render_frame}")