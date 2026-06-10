"""
HMMWV with GPS and IMU Sensors — PyChrono 9.0.x / Irrlicht

Models an HMMWV vehicle driving on a rigid flat terrain with an IMU
(accelerometer) and a GPS sensor attached to the chassis.  The
simulation uses an NSC contact system, a RigidTerrain patch, an
interactive Irrlicht driver, and a ChSensorManager that processes both
sensors each physics step.  Expected behaviour: vehicle accelerates
forward while the sensor manager streams GPS coordinates and IMU
acceleration data each step.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Data paths (required for every catalog-vehicle truth) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation constants ===
step_size       = 1e-3          # physics time step (s)
sim_end         = 30.0          # simulation end time (s)
render_fps      = 50.0          # Irrlicht render cadence (Hz)
render_every    = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once
TERRAIN_LENGTH  = 200.0
TERRAIN_WIDTH   = 200.0
INIT_LOC        = chrono.ChVector3d(0, 0, 0.5)    # chassis spawn
INIT_ROT        = chrono.QuatFromAngleZ(0.0)       # heading east (+X)
GPS_UPDATE_RATE = 10.0          # GPS sensor update rate (Hz)
IMU_UPDATE_RATE = 10.0          # IMU / accelerometer update rate (Hz)
GPS_REF         = chrono.ChVector3d(-89.400, 43.070, 260.0)  # (lon, lat, alt) ref

# === Vehicle setup (HMMWV_Full — NSC for rigid terrain) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)         # TMEASY for stable contact
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                         # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                    # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain below
# joints: suspension + steering links created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Set visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain (RigidTerrain — flat, NSC material) ===
terrain     = veh.RigidTerrain(system)
patch_mat   = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Sensor manager ===
manager = sens.ChSensorManager(system)
# GPS and IMU do not render — no lights needed

# --- IMU (ChAccelerometerSensor) ---
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(-1.5, 0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
imu_sensor = sens.ChAccelerometerSensor(
    chassis,
    IMU_UPDATE_RATE,
    imu_offset,
    sens.ChNoiseNone(),
)
imu_sensor.SetName("IMU Sensor")
imu_sensor.SetLag(0)
imu_sensor.SetCollectionWindow(0)
imu_sensor.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu_sensor)

# --- GPS (ChGPSSensor) ---
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps_sensor = sens.ChGPSSensor(
    chassis,
    GPS_UPDATE_RATE,
    gps_offset,
    GPS_REF,
    sens.ChNoiseNone(),
)
gps_sensor.SetName("GPS Sensor")
gps_sensor.SetLag(0)
gps_sensor.SetCollectionWindow(0)
gps_sensor.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps_sensor)

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with GPS and IMU Sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                           # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive — scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0    # s to go 0 -> +1 steering
throttle_time = 1.0    # s to go 0 -> +1 throttle
braking_time  = 0.3    # s to go 0 -> +1 brake
render_step_size = 1.0 / render_fps
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize all subsystems — order: driver → terrain → vehicle → vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)       # advances wrapper-owned ChSystem
        vis.Advance(step_size)

        # Update sensor manager — once per step after all Advance calls
        manager.Update()


        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
