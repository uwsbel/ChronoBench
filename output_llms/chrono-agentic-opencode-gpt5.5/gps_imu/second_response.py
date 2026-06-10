"""HMMWV GPS/IMU sensor simulation on rigid NSC terrain.

The model uses a catalog HMMWV with Bullet contact against a flat rigid terrain
patch. GPS and accelerometer sensors ride on the chassis, while a scripted
driver applies time-based throttle, steering, and braking so the logged GPS
trajectory shows moving vehicle behavior and braking after six seconds.
"""

import csv
import math

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === simulation and sensor rates are fixed once for repeatable logging
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.002
SIM_END = 8.0
RENDER_FPS = 25.0
LOG_STEP_SIZE = 0.1
GPS_UPDATE_RATE = 10.0
IMU_UPDATE_RATE = 10.0
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.8)
INIT_ROT = chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 0, 1))
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)
SENSOR_OFFSET = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.0), chrono.QUNIT)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
LOG_STEPS = max(1, round(LOG_STEP_SIZE / STEP_SIZE))  # precomputed once


class TimeProgrammedDriver(veh.ChDriver):
    """Time-based driver commands with braking after six seconds."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
        elif time < 2.5:
            self.SetThrottle(0.55)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
        elif time < 4.5:
            self.SetThrottle(0.50)
            self.SetSteering(0.20)
            self.SetBraking(0.0)
        elif time < 6.0:
            self.SetThrottle(0.45)
            self.SetSteering(-0.18)
            self.SetBraking(0.0)
        else:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.75)


def vector_tuple(vec):
    if hasattr(vec, "x"):
        return (float(vec.x), float(vec.y), float(vec.z))
    return (float(vec[0]), float(vec[1]), float(vec[2]))


# === Vehicle and system === wrapper creates the NSC system and vehicle bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: fetched once, reused by sensors and logs
veh_model = vehicle.GetVehicle()  # cache: vehicle subsystem handle reused in setup
# bodies: chassis, suspension links, wheels, and tires are created by HMMWV_Full
# joints: steering and suspension constraints are created by the wrapper
print("VEHICLE MASS: ", veh_model.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid patch supplies flat contact support for the vehicle
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


# === Sensors === GPS and accelerometer ride on the physical chassis body
manager = sens.ChSensorManager(system)
gps = sens.ChGPSSensor(
    chassis,
    GPS_UPDATE_RATE,
    SENSOR_OFFSET,
    GPS_REFERENCE,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

imu = sens.ChAccelerometerSensor(
    chassis,
    IMU_UPDATE_RATE,
    SENSOR_OFFSET,
    sens.ChNoiseNone(),
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)


# === Visualization === vehicle Irrlicht view follows the moving chassis
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS and IMU Logging")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_model)

driver = TimeProgrammedDriver(veh_model)
driver.Initialize()
realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop === synchronize vehicle, terrain, visualization, and sensors
gps_data = []
step_number = 0


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()  # cache: current command reused across subsystem sync
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()

            if step_number % LOG_STEPS == 0:
                gps_buffer = gps.GetMostRecentGPSBuffer()
                if gps_buffer.HasData():  # guard: GPS has no sample before its first tick
                    gps_vec = gps_buffer.GetGPSData()
                    gps_data.append(vector_tuple(gps_vec))


            step_number += 1
            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid sensor data
    print(f"Simulation runtime failure: {exc}")
    raise
except (OSError, IOError) as exc:  # output path or recording file failure
    print(f"Simulation output failure: {exc}")
    raise
finally:
    pass

print("GPS Data: ", gps_data)
