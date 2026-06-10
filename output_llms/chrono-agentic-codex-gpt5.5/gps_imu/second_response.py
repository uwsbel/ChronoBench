"""HMMWV GPS and IMU logging demo on rigid terrain.

This PyChrono 9.0 NSC simulation drives a catalog HMMWV over a flat rigid
terrain patch with scripted throttle, steering, and braking commands. A GPS
sensor and chassis-mounted accelerometer produce buffered readings; GPS samples
are collected at a fixed logging interval and printed after the run.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 0.002
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
LOG_STEP_SIZE = 0.1
LOG_STEPS = max(1, round(LOG_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 220.0
TERRAIN_WIDTH = 60.0
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.7)
INIT_ROT = chrono.QUNIT
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)
SENSOR_RATE = 10.0


class ManeuverDriver(veh.ChDriver):
    """Time-based driver for throttle, steering, and braking commands."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
        elif time < 3.0:
            self.SetThrottle(0.55)
            self.SetSteering(0.18)
            self.SetBraking(0.0)
        elif time < 6.0:
            self.SetThrottle(0.38)
            self.SetSteering(-0.12)
            self.SetBraking(0.0)
        else:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.65)


def gps_tuple(sample):
    if hasattr(sample, "Latitude"):
        return sample.Latitude, sample.Longitude, sample.Altitude
    flat = sample.reshape(-1)
    return float(flat[0]), float(flat[1]), float(flat[2])


def accel_tuple(sample):
    if hasattr(sample, "X"):
        return sample.X, sample.Y, sample.Z
    flat = sample.reshape(-1)
    return float(flat[0]), float(flat[1]), float(flat[2])


# === Vehicle and terrain ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()
system = hmmwv.GetSystem()  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

chassis = hmmwv.GetChassisBody()  # cache: reused by sensors and logging
vehicle = hmmwv.GetVehicle()  # cache: reused by driver and diagnostics
# Wrapper-created components: system, chassis, axles, wheels, tires, steering,
# suspension, and drivetrain are created internally by veh.HMMWV_Full.

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 140, 40)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Sensors ===
manager = sens.ChSensorManager(system)
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.2),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
gps = sens.ChGPSSensor(chassis, SENSOR_RATE, gps_offset, GPS_REFERENCE, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

imu = sens.ChAccelerometerSensor(chassis, SENSOR_RATE, gps_offset, sens.ChNoiseNone())
imu.SetName("IMU Accelerometer")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)


# === Visualization and driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("GPS and IMU HMMWV")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = ManeuverDriver(vehicle)
driver.Initialize()
realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===
gps_data = []
imu_data = []
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()

            if step_number % LOG_STEPS == 0:
                gps_buffer = gps.GetMostRecentGPSBuffer()
                if gps_buffer.HasData():  # guard: GPS buffer is empty before first update
                    gps_sample = gps_buffer.GetGPSData()
                    gps_lat, gps_lon, gps_alt = gps_tuple(gps_sample)
                    gps_data.append(
                        (
                            time,
                            gps_lat,
                            gps_lon,
                            gps_alt,
                        )
                    )
                accel_buffer = imu.GetMostRecentAccelBuffer()
                if accel_buffer.HasData():  # guard: accelerometer buffer is empty before first update
                    accel_sample = accel_buffer.GetAccelData()
                    accel_x, accel_y, accel_z = accel_tuple(accel_sample)
                    imu_data.append((time, accel_x, accel_y, accel_z))


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid sensor state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # file or display failures during recording
    traceback.print_exc()
    raise
finally:
    print("GPS Data: ", gps_data)
