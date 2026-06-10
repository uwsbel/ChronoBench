"""
GPS/IMU Vehicle Sensor Simulation — PyChrono 9.0.x, NSC, Irrlicht

Models an HMMWV vehicle on rigid terrain with IMU (accelerometer) and GPS
sensors mounted on the chassis. The simulation runs in real time, synchronized
with the vehicle subsystem stack (driver, terrain, vehicle, vis). Sensor data
(acceleration, GPS position) is updated each step via a ChSensorManager and
output via access filters.

System: ChSystemNSC (owned by HMMWV_Full wrapper)
Bodies: HMMWV chassis + suspension/wheel sub-bodies, rigid terrain patch
Sensors: ChAccelerometerSensor (IMU), ChGPSSensor (GPS) attached to chassis
Expected behavior: Vehicle renders on flat terrain, sensors report chassis
                   acceleration and GPS coordinates throughout the run.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Constants ===
step_size = 1e-3          # physics time step (s)
sim_end   = 20.0          # simulation duration (s)
render_fps = 50.0
render_steps = math.ceil(1.0 / (render_fps * step_size))  # precomputed once

# Vehicle init pose
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(0.0)

# Terrain size
TERRAIN_LENGTH = 600.0
TERRAIN_WIDTH  = 600.0

# Sensor offsets on chassis
IMU_OFFSET_POS = chrono.ChVector3d(-1.0, 0.0, 0.3)
GPS_OFFSET_POS = chrono.ChVector3d(-1.0, 0.0, 0.3)
SENSOR_UPDATE_RATE = 10.0   # Hz — physical rate for both IMU and GPS

# === Data paths (mandatory for catalog-vehicle truth) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                    # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()               # main chassis rigid body; cache: fetched once, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain patch body below
# joints: suspension + steering created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (after Initialize) — veh.VisualizationType_* in this 9.0.0 build
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht visualization (vehicle-specific) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS/IMU Sensor Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / render_fps   # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensors (IMU + GPS attached to chassis) ===
manager = sens.ChSensorManager(system)

imu_offset = chrono.ChFramed(
    IMU_OFFSET_POS,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps_offset = chrono.ChFramed(
    GPS_OFFSET_POS,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

# IMU (accelerometer)
imu = sens.ChAccelerometerSensor(chassis, SENSOR_UPDATE_RATE, imu_offset, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# GPS
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)  # ref lat/lon/alt
gps = sens.ChGPSSensor(chassis, SENSOR_UPDATE_RATE, gps_offset, GPS_REFERENCE, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        manager.Update()


        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
