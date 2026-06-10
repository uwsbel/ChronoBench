import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Set the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH                            # mesh visualization
chassis_collision_type = veh.CollisionType_NONE                  # no chassis collision
tire_model = veh.TireModelType_TMEASY                            # TMEASY tire model

terrainHeight = 0                                                 # flat terrain
terrainLength = 100.0                                             # terrain size X
terrainWidth = 100.0                                              # terrain size Y

trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)                  # chase camera target

contact_method = chrono.ChContactMethod_NSC                      # NSC contact

step_size = 1e-3                                                  # simulation step size
tire_step_size = step_size                                        # tire step size
render_step_size = 1.0 / 50                                      # 50 FPS
log_step_size = 1.0 / 20                                         # 20 Hz data logging
sim_end = 20.0                                                    # simulation end time

render_fps = 50
render_every = max(1, round(1.0 / (render_fps * step_size)))     # render cadence

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # bullet collision

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
vis.SetWindowTitle('HMMWV GPS/IMU Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system with constant steering 0.6 and throttle 0.5
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                               # time to go 0 -> +1
throttle_time = 1.0                                               # time to go 0 -> +1
braking_time = 0.3                                                # time to go 0 -> +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# IMU sensor offset pose: offset at (0, 0, 1) as modified in turn3
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

# Create IMU (accelerometer) sensor
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(),        # attached to chassis
                                 10,                              # update rate Hz
                                 offset_pose,                     # offset pose
                                 sens.ChNoiseNone())              # no noise
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())                        # host access to IMU data
manager.AddSensor(imu)

# Create GPS sensor with same offset pose
gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                  # attached to chassis
                       10,                                        # update rate Hz
                       offset_pose,                               # offset pose
                       chrono.ChVector3d(-89.400, 43.070, 260.0), # GPS reference lat/lon/alt
                       sens.ChNoiseNone())                        # no noise
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())                          # host access to GPS data
manager.AddSensor(gps)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)            # steps between renders
log_steps = math.ceil(log_step_size / step_size)                  # steps between log events
realtime_timer = chrono.ChRealtimeStepTimer()                     # real-time synchronizer
step_number = 0
render_frame = 0


gps_data = []                                                     # collect GPS trajectory data

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    if step_number % log_steps == 0:
        gps_buf = gps.GetMostRecentGPSBuffer()
        if gps_buf.HasData():
            gps_coor = gps_buf.GetGPSData()
            gps_data.append([gps_coor[0], gps_coor[1], gps_coor[2]])

    # Set constant driver inputs: steering=0.6, throttle=0.5
    driver.SetSteering(0.6)                                       # constant steering
    driver.SetThrottle(0.5)                                       # constant throttle
    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    manager.Update()                                              # update all sensors

    step_number += 1
    realtime_timer.Spin(step_size)

print("GPS Data: ", gps_data)                                     # print GPS trajectory
