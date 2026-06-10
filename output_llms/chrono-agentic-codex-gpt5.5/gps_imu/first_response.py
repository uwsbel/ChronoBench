"""HMMWV GPS/IMU sensor simulation on rigid terrain.

This PyChrono 9.0 NSC vehicle scene builds a full HMMWV wrapper system, a
rigid terrain patch, an Irrlicht vehicle visual interface, and GPS plus IMU
sensors attached to the chassis. The vehicle advances under driver inputs while
the terrain, vehicle, visual system, and sensor manager are synchronized every
step; sensor buffers are checked and processed as data becomes available.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === simulation timing, terrain, and vehicle start values
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 120.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)
SENSOR_RATE = 10.0


# === Vehicle and system === HMMWV wrapper owns the NSC dynamics system
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: vehicle interface reused for mass, vis, and driver
chassis = hmmwv.GetChassisBody()  # cache: chassis body hosts GPS and IMU sensors
print("VEHICLE MASS: ", vehicle.GetMass())


# === Terrain === rigid contact patch for vehicle support and tire forces
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization === vehicle Irrlicht interface with chase camera and light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS and IMU Sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Driver === interactive driver for real-time vehicle inputs
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Sensors === GPS and accelerometer mounted to the HMMWV chassis
manager = sens.ChSensorManager(system)

sensor_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)

imu = sens.ChAccelerometerSensor(chassis, SENSOR_RATE, sensor_pose, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

gps = sens.ChGPSSensor(
    chassis,
    SENSOR_RATE,
    sensor_pose,
    GPS_REFERENCE,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)


def process_sensor_buffers(current_time):
    """Read GPS and IMU buffers only after each sensor has produced data."""
    latest_accel = None
    latest_gps = None

    accel_buf = imu.GetMostRecentAccelBuffer()
    if accel_buf.HasData():  # guard: sensor buffers are empty before the first tick
        latest_accel = accel_buf.GetAccelData()

    gps_buf = gps.GetMostRecentGPSBuffer()
    if gps_buf.HasData():  # guard: process GPS only after the filter publishes data
        latest_gps = gps_buf.GetGPSData()

    if latest_accel is not None and latest_gps is not None:
        if int(current_time * 10.0) != int((current_time - STEP_SIZE) * 10.0):
            print("SENSOR DATA: ", current_time, latest_gps, latest_accel)

    return latest_gps, latest_accel


# === Main loop === synchronize driver, terrain, vehicle, visual system, and sensors
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()


            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()
            process_sensor_buffers(system.GetChTime())

            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # guard: Chrono runtime or invalid-state errors
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # guard: output capture errors during review recording
    traceback.print_exc()
    raise
finally:
    pass
