"""
GPS/IMU sensor demo — HMMWV driving on SCM terrain with GPS and IMU sensors.

plan_type: mbs_in_scene (wheeled vehicle + sensors)
Modifications (turn 2):
  - Added log_step_size to control data logging frequency
  - Added gps_data list to store GPS coordinates
  - GPS data logging at intervals via gps.GetMostRecentGPSBuffer().GetGPSData()
  - Time-based driver inputs: throttle/steering schedule + braking after 6 s
  - Print logged GPS data at end of simulation
"""

import os
import math
import numpy as np

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr


# === Physical constants ===
time_step = 1e-3
sim_end = 12.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Logging control (turn 2 modification) ===
log_step_size = 0.05    # seconds between data logging
log_steps = max(1, round(log_step_size / time_step))

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle (HMMWV Full) on NSC rigid terrain ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()
system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain (RigidTerrain NSC) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver — time-based control (turn 2 modification: scripted + braking at 6 s) ===
class TimedDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 1.0:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        elif time < 4.0:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        elif time < 6.0:
            self.SetThrottle(0.3)
            self.SetBraking(0.0)
            self.SetSteering(0.15)
        elif time < 8.0:
            self.SetThrottle(0.2)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(0.0)
            self.SetBraking(0.8)
            self.SetSteering(0.0)

driver = TimedDriver(hmmwv.GetVehicle())
driver.Initialize()

# === Vehicle Visualization Types (set before AttachVehicle) ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("GPS/IMU Demo — HMMWV")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Sensor Manager ===
manager = sens.ChSensorManager(system)

# IMU sensor (accelerometer on chassis)
imu_body = hmmwv.GetChassisBody()
imu_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT)
imu = sens.ChAccelerometerSensor(imu_body, 10, imu_offset, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# GPS sensor
gps_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT)
gps = sens.ChGPSSensor(
    imu_body, 10, gps_offset,
    chrono.ChVector3d(-89.400, 43.070, 260.0),
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === GPS data list (turn 2 modification) ===
gps_data = []

# === CSV logging (review-only) ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))

os.makedirs("frames", exist_ok=True)
frame = 0
log_counter = 0

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)
        manager.Update()

        # GPS data logging at intervals (turn 2 modification)
        log_counter += 1
        if log_counter >= log_steps:
            log_counter = 0
            gps_buffer = gps.GetMostRecentGPSBuffer()
            if gps_buffer.HasData():
                gps_reading = list(gps_buffer.GetGPSData())
                gps_data.append(gps_reading)

        if system.GetChTime() >= sim_end:
            break

    step_number += 1
    realtime_timer.Spin(time_step)

# Print logged GPS data (turn 2 modification)
print("GPS Data: ", gps_data)

# === Review-only: assemble videos + plot ===
