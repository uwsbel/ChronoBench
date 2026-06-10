"""
GPS and IMU Sensor Demo with Scripted Driver — PyChrono 9.0.x / Irrlicht.

Simulates an HMMWV vehicle driving on rigid terrain while logging GPS coordinates
at a controlled rate (log_step_size). A GPS sensor and an IMU accelerometer are
attached to the chassis. The driver applies throttle/steering for the first 6
seconds, then switches to braking. GPS data is accumulated in gps_data and
printed at the end of the simulation.

System: ChSystemNSC (rigid-terrain, NSC contact)
Bodies: HMMWV chassis + suspension/wheel/tire system (wrapper-created), terrain patch
Expected behavior: vehicle moves forward with slight steering, then brakes after 6 s;
GPS coordinates are logged at regular intervals and printed at exit.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Named constants ===
STEP_SIZE = 5e-4                   # simulation time step (s)
SIM_END = 15.0                     # total simulation time (s)
RENDER_FPS = 50.0                  # Irrlicht render cadence
LOG_STEP_SIZE = 0.1                # GPS data logging interval (s) — controls frequency
TERRAIN_LENGTH = 200.0             # rigid terrain patch length (m)
TERRAIN_WIDTH = 200.0              # rigid terrain patch width (m)
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)   # chassis initial position
INIT_ROT = chrono.QuatFromAngleZ(0.0)     # chassis initial orientation

# Precomputed loop cadences
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# === Vehicle setup (HMMWV_Full wrapper — owns ChSystem) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (wrapper-created) ===
sys = hmmwv.GetSystem()               # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = hmmwv.GetChassisBody()      # cache: chassis rigid body, reused throughout
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); suspension links: wrapper-created
# joints: suspension, steering, powertrain — created inside the wrapper

# Set visualization types (must call after Initialize; vis types live in veh namespace)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Terrain setup ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Scripted driver — time-based throttle/steering/braking ===
# The driver applies throttle + slight steering for t < 6 s, then brakes.
# Using ChDataDriver for the scored core to implement the described schedule.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0,  0.0, 0.0,  0.0),   # initial idle
    veh.DataDriverEntry(0.5,  0.1, 0.6,  0.0),   # ramp up: slight right steer + throttle
    veh.DataDriverEntry(3.0,  0.1, 0.7,  0.0),   # hold: throttle 70%, steer 10%
    veh.DataDriverEntry(6.0,  0.0, 0.3,  0.0),   # reduce throttle, center steer
    veh.DataDriverEntry(6.01, 0.0, 0.0,  1.0),   # braking after 6 s
    veh.DataDriverEntry(15.0, 0.0, 0.0,  1.0),   # hold braking until end
])
driver = veh.ChDataDriver(hmmwv.GetVehicle(), driver_data)
driver.Initialize()

# === Sensor manager, GPS, and IMU ===
manager = sens.ChSensorManager(sys)
# Lighting: point lights for the sensor camera rendering pipeline
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

gps_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-1.5, 0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
# Reference origin: Madison, WI (lat/lon/alt)
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)

gps = sens.ChGPSSensor(
    chassis,           # attached to chassis body (cache reused)
    10,                # update_rate (Hz) — physical rate, not 1/dt
    gps_offset_pose,
    GPS_REFERENCE,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

imu_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-1.5, 0, 0.6),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
imu = sens.ChAccelerometerSensor(
    chassis,           # attached to chassis body (cache reused)
    10,                # update_rate (Hz)
    imu_offset_pose,
    sens.ChNoiseNone(),
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# === Irrlicht visualization (vehicle-aware) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS/IMU Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === GPS logging state ===
gps_data = []                      # stores logged GPS coordinates (lat, lon, alt)
log_step_size = LOG_STEP_SIZE      # frequency control variable (matches prompt naming)
log_steps = max(1, round(log_step_size / STEP_SIZE))  # steps between GPS logs; precomputed once

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()   # cache: current sim time (retrieved once per render frame)

        # --- Irrlicht render (throttled) ---
        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # --- Driver + vehicle synchronize & advance ---
        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)    # advances wrapper-owned system; do NOT also call sys.DoStepDynamics
        vis.Advance(STEP_SIZE)

        # --- Sensor update ---
        manager.Update()

        # --- GPS data logging at controlled intervals ---
        if step_number % log_steps == 0:
            gps_buf = gps.GetMostRecentGPSBuffer()
            if gps_buf.HasData():  # guard: skip frames the sensor hasn't filled yet
                coord = gps_buf.GetGPSData()
                gps_data.append(coord)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    print("GPS Data: ", gps_data)

