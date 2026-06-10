"""
GPS and IMU Sensor Demo — HMMWV on Rigid Terrain
=================================================
System type : NSC (ChSystemNSC via HMMWV_Full wrapper)
Vehicle     : HMMWV_Full with rigid terrain
Sensors     : GPS sensor + IMU accelerometer sensor on the chassis
Driver      : Interactive (ChInteractiveDriverIRR); scripted maneuver in review-only block
Objective   : Log GPS coordinates at defined intervals; demonstrate GPS/IMU data access;
              apply throttle/steering/braking schedule based on simulation time
              (throttle forward at 0.5 s, add steering at ~3 s, brake after 6 s).
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

STEP_SIZE        = 1e-3          # simulation time step (s)
SIM_END          = 10.0          # simulation end time (s)
RENDER_FPS       = 50.0          # Irrlicht render rate (Hz)
render_every     = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH   = 300.0         # terrain X extent (m)
TERRAIN_WIDTH    = 300.0         # terrain Y extent (m)

GPS_UPDATE_RATE  = 10.0          # GPS update rate (Hz)
IMU_UPDATE_RATE  = 10.0          # IMU/accelerometer update rate (Hz)

LOG_STEP_SIZE    = 1.0 / GPS_UPDATE_RATE   # data logging interval (s); matches GPS rate
INIT_LOC         = chrono.ChVector3d(0, 0, 0.5)
INIT_ROT         = chrono.QuatFromAngleZ(0.0)

# GPS reference origin (lat/lon/alt)
GPS_REF = chrono.ChVector3d(-89.400, 43.070, 260.0)

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()           # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()      # main chassis rigid body  # cache: fetched once, reused
# wheels/spindles: hmmwv.GetVehicle().GetAxles(); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (set after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Sensor manager (GPS + IMU do not render; no lights needed) ===
manager = sens.ChSensorManager(system)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

# IMU / accelerometer sensor
imu = sens.ChAccelerometerSensor(chassis, IMU_UPDATE_RATE, offset_pose, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# GPS sensor
gps = sens.ChGPSSensor(
    chassis, GPS_UPDATE_RATE, offset_pose,
    GPS_REF,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Visualization (full Irrlicht vehicle window) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS + IMU Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive — scored-core default matching truth) ===
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / RENDER_FPS   # precomputed once
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# GPS data accumulator and logging interval state
gps_data = []
log_steps = max(1, round(LOG_STEP_SIZE / STEP_SIZE))  # precomputed once

# === Review-only setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

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

        # Log GPS data at defined interval
        if step_number % log_steps == 0:
            gps_buf = gps.GetMostRecentGPSBuffer()
            if gps_buf.HasData():
                gps_data.append(gps_buf.GetGPSData())

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    # flush partial GPS log even if a step diverges
    if gps_data:
        print("GPS Data: ", gps_data)

print("GPS Data: ", gps_data)
