"""HMMWV GPS/IMU vehicle simulation.

This script builds a PyChrono NSC HMMWV_Full vehicle on rigid terrain, drives it
with scripted driver inputs, renders the catalog vehicle through Irrlicht, and
updates GPS plus IMU channels attached to the chassis. The expected behavior is a
forward-moving HMMWV whose chassis pose feeds live GPS, accelerometer, and
gyroscope buffers while the vehicle, terrain, driver, and visual systems are
synchronized and advanced together.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Parameters === named constants define the run length, terrain, and sensor rates
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 0.002
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
SUPPORT_TOP_Z = 0.0
SUSPENSION_REF_HEIGHT = 0.5
TIRE_RADIUS = 0.47
WHEEL_Z_TOL = 0.10

INIT_LOC = chrono.ChVector3d(0.0, 0.0, SUPPORT_TOP_Z + SUSPENSION_REF_HEIGHT)
INIT_ROT = chrono.QUNIT
SENSOR_RATE = 10.0
SENSOR_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.2),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 0, 1)),
)
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)


# === Vehicle and system === HMMWV wrapper owns the contact system and vehicle bodies
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: full vehicle handle reused for mass, speed, and spindles
chassis = hmmwv.GetChassisBody()  # cache: chassis body carries GPS and IMU sensors
# bodies: chassis, wheels, tires, suspension, and steering are created by HMMWV_Full.
# joints: suspension, steering, and driveline constraints are created inside the wrapper.
print("VEHICLE MASS: ", vehicle.GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

spindle_world = []
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= SUPPORT_TOP_Z - WHEEL_Z_TOL, (
    f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs support top z={SUPPORT_TOP_Z:.3f}"
)


# === Terrain === rigid flat terrain gives the HMMWV a paved contact surface
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Sensors === manager updates GPS and IMU buffers attached to the chassis
manager = sens.ChSensorManager(system)

imu_accel = sens.ChAccelerometerSensor(chassis, SENSOR_RATE, SENSOR_OFFSET, sens.ChNoiseNone())
imu_accel.SetName("IMU Accelerometer")
imu_accel.SetLag(0)
imu_accel.SetCollectionWindow(0)
imu_accel.SetOffsetPose(SENSOR_OFFSET)
imu_accel.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu_accel)

imu_gyro = sens.ChGyroscopeSensor(chassis, SENSOR_RATE, SENSOR_OFFSET, sens.ChNoiseNone())
imu_gyro.SetName("IMU Gyroscope")
imu_gyro.SetLag(0)
imu_gyro.SetCollectionWindow(0)
imu_gyro.SetOffsetPose(SENSOR_OFFSET)
imu_gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(imu_gyro)

gps = sens.ChGPSSensor(chassis, SENSOR_RATE, SENSOR_OFFSET, GPS_REFERENCE, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.SetOffsetPose(SENSOR_OFFSET)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)


# === Visualization and driver === Irrlicht vehicle view plus scripted driver inputs
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS IMU")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)

driver_data = veh.vector_Entry()
driver_data.append(veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0))
driver_data.append(veh.DataDriverEntry(0.5, 0.0, 0.4, 0.0))
driver_data.append(veh.DataDriverEntry(2.5, 0.12, 0.45, 0.0))
driver_data.append(veh.DataDriverEntry(4.0, -0.10, 0.35, 0.0))
driver_data.append(veh.DataDriverEntry(SIM_END, 0.0, 0.25, 0.0))
driver = veh.ChDataDriver(vehicle, driver_data)
driver.Initialize()

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Main loop === synchronize and advance vehicle, terrain, driver, visual, and sensors
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0
latest_gps = (0.0, 0.0, 0.0)
latest_accel = (0.0, 0.0, 0.0)
latest_gyro = (0.0, 0.0, 0.0)

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        time = system.GetChTime()
        driver_inputs = driver.GetInputs()  # cache: consumed by vehicle and visual synchronization

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()

        gps_buffer = gps.GetMostRecentGPSBuffer()
        if gps_buffer.HasData():  # guard: GPS has no sample until its first sensor tick
            gps_data = gps_buffer.GetGPSData()
            latest_gps = (float(gps_data[0]), float(gps_data[1]), float(gps_data[2]))

        accel_buffer = imu_accel.GetMostRecentAccelBuffer()
        if accel_buffer.HasData():  # guard: accelerometer sample may lag the physics step
            accel_data = accel_buffer.GetAccelData()
            latest_accel = (float(accel_data[0]), float(accel_data[1]), float(accel_data[2]))

        gyro_buffer = imu_gyro.GetMostRecentGyroBuffer()
        if gyro_buffer.HasData():  # guard: gyroscope sample may lag the physics step
            gyro_data = gyro_buffer.GetGyroData()
            latest_gyro = (float(gyro_data[0]), float(gyro_data[1]), float(gyro_data[2]))

        pos = chassis.GetPos()
        speed = vehicle.GetSpeed()

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    raise
except (OSError, IOError) as exc:  # filesystem errors from review frame or CSV output
    raise
finally:
    pass
