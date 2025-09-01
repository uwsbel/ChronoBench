import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import matplotlib.pyplot as plt  # Required for plotting

# Set the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render and log frames
render_step_size = 1.0 / 50  # 50 FPS
log_step_size = 1.0 / 20     # 20 Hz logging

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle inputs
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Create and configure IMU sensor
offset_pose_imu = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
imu = sens.ChAccelerometerSensor(
    vehicle.GetChassisBody(),
    10,  # Update rate in Hz
    offset_pose_imu,
    sens.ChNoiseNone()
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# Create and configure GPS sensor
offset_pose_gps = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
gps = sens.ChGPSSensor(
    vehicle.GetChassisBody(),
    10,  # Update rate in Hz
    offset_pose_gps,
    chrono.ChVector3d(-89.400, 43.070, 260.0),  # Reference point
    sens.ChNoiseNone()
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# Simulation loop setup
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
log_steps = math.ceil(log_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

gps_data = []

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Log GPS data
    if step_number % log_steps == 0:
        gps_coor = gps.GetMostRecentGPSBuffer().GetGPSData()
        gps_data.append([gps_coor[0], gps_coor[1], gps_coor[2]])

    # Set constant driver inputs
    driver.SetThrottle(0.5)
    driver.SetSteering(0.6)

    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update sensors
    manager.Update()

    step_number += 1
    realtime_timer.Spin(step_size)

# Output GPS data
print("GPS Data: ", gps_data)

# Plot GPS trajectory
if gps_data:
    latitudes = [point[0] for point in gps_data]
    longitudes = [point[1] for point in gps_data]

    plt.figure(figsize=(10, 6))
    plt.plot(longitudes, latitudes, 'b-', label='Vehicle Path')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Vehicle GPS Trajectory')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()
else:
    print("No GPS data collected.")