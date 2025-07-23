import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Set the HMMWV vehicle path
hmmwv_path = chrono.GetChronoDataPath() + 'vehicle/hmmwv'

# Set the data path for the simulation
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()(True)
vehicle.SetDataPath(hmmwv_path + '/data')
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetFixed(False)

# Set the collision system type
vehicle.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set the drivetrain type and terrain type
vehicle.SetDrivetrainType(veh.DrivetrainType_SHAFTS)
vehicle.SetTerrainType(veh.TerrainType_PLANE)

# Set the contact model for the tires
vehicle.SetTireType(veh.TireType_TMEASY)
vehicle.SetTireStepSize(1e-3)

# Set the maximum wheel load Ratio
vehicle.SetMaxWheelLoadRatio(0.9)

# Initialize the vehicle
vehicle.Initialize()

# Create the Chrono interface for the vehicle
vehicle_interface = veh.ChWheeledVehicleInterfaceChrono()
vehicle_interface.SetVehicle(vehicle.GetVehicle())

# -----------------------
# Create the sensor manager
# -----------------------
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())

# Create an IMU sensor and add it to the sensor manager
offset_pose = chrono.ChPose()
offset_pose.pos = chrono.ChVector3d(-10, 0, 1)
imu_sensor = sens.ChAccelerometerSensor(vehicle.GetChassisBody(),                     # Body IMU is attached to
                                        10,        # Update rate in Hz
                                        offset_pose,  # Offset pose
                                        sens.ChNoiseNone())  # Noise model
imu_sensor.SetName("IMU Sensor")
imu_sensor.SetLag(0)
imu_sensor.SetCollectionWindow(0)
# Provides the host access to the IMU data
imu_sensor.PushFilter(sens.ChFilterAccelAccess())
# Add the IMU sensor to the sensor manager
sensor_manager.AddSensor(imu_sensor)

# Create a GPS sensor and add it to the sensor manager
gps_sensor = sens.ChGPSSensor(vehicle.GetChassisBody(),                     # Body GPS is attached to
                              10,        # Update rate in Hz
                              offset_pose,  # Offset pose
                              chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference point
                              sens.ChNoiseNone())  # Noise model
gps_sensor.SetName("GPS Sensor")
gps_sensor.SetLag(0)
gps_sensor.SetCollectionWindow(0)
# Provides the host access to the GPS data
gps_sensor.PushFilter(sens.ChFilterGPSAccess())
# Add the GPS sensor to the sensor manager
sensor_manager.AddSensor(gps_sensor)

# ---------------------
# Create the Irrlicht app
# ---------------------
vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 720)
vis.SetChanels(32)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetVehicle())

# -----------------------
# Simulation parameters
# -----------------------
time_step = 1e-3             # Simulation time step
end_time = 100.0             # End of simulation time
time_interval = 1.0          # Time interval between two rendering frames

# Set the simulation time step for the vehicle
vehicle.SetTimeStep(time_step)

# run the simulation
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = vehicle.GetSoftStopTime()
while vis.Run() :
    time = vehicle.GetTime()
    # Render the scene
    if time - time_interval >= 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        time_interval += 1

    # Update the sensor manager in each step
    sensor_manager.Update()
    # Synchronize all modules (with current frame time)
    vehicle.Synchronize(time)
    sensor_manager.Synchronize(time)
    # Advance the simulation for one timestep for all modules
    vehicle.Advance(time_step)
    sensor_manager.Advance(time_step)
    # Spin in place for real time to catch up
    realtime_timer.Spin(step)