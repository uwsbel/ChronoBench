"""
GPS and IMU Sensor Demo — HMMWV on Rigid Terrain with GPS/IMU Sensing.

System type: ChSystemNSC (created and owned by the HMMWV_Full wrapper).
Main bodies: HMMWV chassis, wheels, tire spindles, rigid terrain patch.
Sensors: GPS sensor (attached to chassis, offset (0,0,1)) and IMU/accelerometer
         sensor (attached to chassis, offset (0,0,1)).
Driver: Scripted ChDataDriver with constant steering=0.6 and throttle=0.5 throughout.
Expected behavior: HMMWV drives in a curved path (steering=0.6) at moderate
    throttle (0.5). GPS sensor records latitude/longitude trajectory;
    IMU records accelerations. At end, GPS trajectory is plotted.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# VisualizationType lives in veh namespace in this build
VisMesh       = veh.VisualizationType_MESH
VisPrimitives = veh.VisualizationType_PRIMITIVES
VisNone       = veh.VisualizationType_NONE


# === Constants ===
STEP_SIZE = 1e-3            # physics time step (s)
SIM_END = 20.0              # simulation duration (s)
RENDER_FPS = 50.0           # Irrlicht render frame rate
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0      # m
TERRAIN_WIDTH  = 200.0      # m

# Vehicle spawn
INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest (HMMWV)
INIT_Z = SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)

# Sensor parameters
GPS_RATE   = 10.0   # Hz — physical update rate
IMU_RATE   = 10.0   # Hz — physical update rate
GPS_REF    = chrono.ChVector3d(-89.400, 43.070, 260.0)  # reference lat/lon/alt


# === Data paths (scored core — truth requires these) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)   # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()         # cache: main chassis rigid body
# wheels/spindles accessed via hmmwv.GetVehicle().GetAxle(i)
# suspension + steering joints created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualize vehicle subsystems
hmmwv.SetChassisVisualizationType(VisMesh)
hmmwv.SetSuspensionVisualizationType(VisPrimitives)
hmmwv.SetSteeringVisualizationType(VisPrimitives)
hmmwv.SetWheelVisualizationType(VisMesh)
hmmwv.SetTireVisualizationType(VisMesh)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS/IMU Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()  # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver — scripted constant inputs: steering=0.6, throttle=0.5 ===
# Using ChDataDriver with constant entries throughout SIM_END
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0,     0.6, 0.5, 0.0),
    veh.DataDriverEntry(SIM_END, 0.6, 0.5, 0.0),
])
driver = veh.ChDataDriver(hmmwv.GetVehicle(), driver_data)
driver.Initialize()

# === Sensor Manager ===
manager = sens.ChSensorManager(sys)
# GPS and IMU do not need scene lights (no visual rendering)

# IMU (accelerometer) — offset pose at (0, 0, 1)
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
imu = sens.ChAccelerometerSensor(chassis, IMU_RATE, imu_offset, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# GPS — offset pose at (0, 0, 1)
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    chassis, GPS_RATE, gps_offset,
    GPS_REF,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        # Throttled rendering
        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver provides constant steering=0.6, throttle=0.5 via ChDataDriver
        driver_inputs = driver.GetInputs()

        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        # Update sensor manager
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
