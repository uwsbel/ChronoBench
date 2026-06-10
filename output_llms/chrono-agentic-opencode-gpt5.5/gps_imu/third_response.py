"""HMMWV rigid-terrain GPS/IMU demo using NSC contact.

The vehicle drives on a flat rigid patch with constant steering and throttle.
GPS and accelerometer sensors ride on the chassis at a centered offset pose,
and the recorded GPS latitude/longitude samples are plotted after the run.
"""

import csv
import math
import traceback

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === simulation timing, vehicle pose, and sensor rates
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.002
SIM_END = 8.0
RENDER_FPS = 25.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.6)
INIT_ROT = chrono.QUNIT
SENSOR_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)
DRIVER_STEERING = 0.6
DRIVER_THROTTLE = 0.5


# === Vehicle & terrain === wrapper creates the system, bodies, and vehicle joints
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned system reused for terrain/sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: chassis body reused for sensors and logging
veh_obj = vehicle.GetVehicle()  # cache: vehicle interface reused for driver/visualization
print("VEHICLE MASS: ", veh_obj.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Sensors === GPS and accelerometer measure the chassis from a centered offset
manager = sens.ChSensorManager(system)
noise = sens.ChNoiseNone()

imu = sens.ChAccelerometerSensor(chassis, 10.0, SENSOR_OFFSET, noise)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

gps = sens.ChGPSSensor(chassis, 10.0, SENSOR_OFFSET, GPS_REFERENCE, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)


# === Driver === constant scripted inputs requested by the task
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, DRIVER_STEERING, DRIVER_THROTTLE, 0.0),
    veh.DataDriverEntry(SIM_END, DRIVER_STEERING, DRIVER_THROTTLE, 0.0),
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()


# === Visualization === Irrlicht vehicle view for real-time display and review video
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("GPS and IMU HMMWV")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)


# === Main loop === synchronize vehicle subsystems and record GPS samples
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
gps_times = []
gps_latitudes = []
gps_longitudes = []
gps_altitudes = []
imu_accel_x = []
imu_accel_y = []
imu_accel_z = []

try:
    with open("gps_data.csv", "w", newline="") as gps_file:
        gps_writer = csv.writer(gps_file)
        gps_writer.writerow([
            "time", "latitude", "longitude", "altitude", "accel_x", "accel_y", "accel_z",
        ])

        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                driver.Synchronize(time)
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                vehicle.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)
                manager.Update()

                gps_buf = gps.GetMostRecentGPSBuffer()
                accel_buf = imu.GetMostRecentAccelBuffer()
                if gps_buf.HasData():
                    gps_data = gps_buf.GetGPSData()
                    accel_x = accel_y = accel_z = 0.0
                    if accel_buf.HasData():
                        accel_data = accel_buf.GetAccelData()
                        accel_x = float(accel_data[0])
                        accel_y = float(accel_data[1])
                        accel_z = float(accel_data[2])
                    latitude = float(gps_data[0])
                    longitude = float(gps_data[1])
                    altitude = float(gps_data[2])
                    gps_times.append(time)
                    gps_latitudes.append(latitude)
                    gps_longitudes.append(longitude)
                    gps_altitudes.append(altitude)
                    imu_accel_x.append(accel_x)
                    imu_accel_y.append(accel_y)
                    imu_accel_z.append(accel_z)
                    gps_writer.writerow([
                        time, latitude, longitude, altitude,
                        accel_x, accel_y, accel_z,
                    ])

                if system.GetChTime() >= SIM_END:
                    break
                realtime_timer.Spin(STEP_SIZE)

except (OSError, IOError) as exc:  # disk or permission errors while writing logs
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # Chrono stepping or invalid sensor data errors
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === prompt-requested latitude/longitude trajectory plot
if gps_latitudes and gps_longitudes:
    plt.figure(figsize=(7, 5))
    plt.plot(gps_longitudes, gps_latitudes, marker="o", linewidth=1.5)
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("GPS trajectory")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("gps_trajectory.png")
    plt.close()
