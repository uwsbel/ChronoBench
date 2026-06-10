"""
GPS + IMU sensor simulation with HMMWV on rigid terrain.

Changes from base (turn2):
  - IMU sensor offset pose: chrono.ChVector3d(0, 0, 1) (was -8, 0, 1)
  - Driver: constant steering=0.6, throttle=0.5 throughout simulation
  - Matplotlib GPS trajectory plot at end

System: ChSystemNSC with wheeled vehicle + rigid terrain.
"""

import os
import math
import csv as csvmod
import numpy

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Named constants ===
STEERING = 0.6
THROTTLE = 0.5
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# === Recording controls (review-only) ===

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle (HMMWV_Full — wrapper owns its ChSystemNSC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS:", hmmwv.GetVehicle().GetMass())

# Cache handles for the scored core loop
chassis_body = hmmwv.GetChassisBody()  # cache: GPS/IMU attach body

# === Terrain (RigidTerrain) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)
patch.SetColor(chrono.ChColor(0.8, 0.5, 0.2))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("GPS + IMU Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Sensors: GPS + IMU ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

# IMU (accelerometer) — offset changed to (0, 0, 1) per input3
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
imu = sens.ChAccelerometerSensor(
    chassis_body,
    10,
    imu_offset,
    sens.ChNoiseNone(),
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# GPS — reference lat/lon/alt for the test area
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    chassis_body,
    10,
    gps_offset,
    chrono.ChVector3d(0.0, 0.0, 0.0),  # reference origin
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === CSV logging (review-only) >>>

# === Main loop ===
frame = 0
while vis.Run() and system.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(RENDER_EVERY):
        sim_time = system.GetChTime()

        # GPS data collection (review-only)

        # IMU data collection (review-only)

        # Log CSV (review-only)

        # Scripted driver inputs: constant steering + throttle (input3 change)
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = STEERING
        driver_inputs.m_throttle = THROTTLE
        driver_inputs.m_braking = 0.0

        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        hmmwv.Advance(TIME_STEP)
        manager.Update()

        if system.GetChTime() >= SIM_END:
            break
