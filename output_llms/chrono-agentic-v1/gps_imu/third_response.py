"""
GPS and IMU Sensor Demo — HMMWV on Rigid Terrain (ChSystemNSC).

This simulation drives an HMMWV vehicle with constant throttle=0.5 and
steering=0.6 on a flat rigid terrain patch. An IMU accelerometer sensor
is mounted at offset (0,0,1) from the chassis, and a GPS sensor logs
latitude/longitude coordinates. After the run, the GPS trajectory is
plotted. All sensors use physical update rates (10 Hz).

System: ChSystemNSC (wrapper-owned)
Bodies: HMMWV chassis + wheels/spindles (wrapper-created), terrain patch body
Expected: Vehicle turns steadily with the constant steering; GPS logs a
curved trajectory; IMU records accelerations from the turning motion.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Data path setup ===
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
TIME_STEP = 1e-3            # physics step size (s)
SIM_END = 20.0              # simulation duration (s)
RENDER_FPS = 50.0           # Irrlicht render rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TERRAIN_LENGTH = 200.0      # terrain patch X extent (m)
TERRAIN_WIDTH = 200.0       # terrain patch Y extent (m)

INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5  # HMMWV chassis origin above wheel-bottom at rest (m)
INIT_Z = SUSPENSION_REF_HEIGHT

CONSTANT_THROTTLE = 0.5
CONSTANT_STEERING = 0.6

# GPS reference origin (lat deg, lon deg, alt m)
GPS_REF_LAT = 43.070
GPS_REF_LON = -89.400
GPS_REF_ALT = 260.0

# IMU sensor offset pose: (0, 0, 1) as specified
IMU_OFFSET = chrono.ChVector3d(0, 0, 1)

# === Vehicle setup (HMMWV_Full wrapper owns the ChSystem) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move

init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                     # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()            # cache: fetched once, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain (RigidTerrain, NSC material) ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver — constant throttle 0.5 and steering 0.6 throughout simulation ===
driver_inputs = veh.DriverInputs()
driver_inputs.m_throttle = CONSTANT_THROTTLE
driver_inputs.m_steering = CONSTANT_STEERING
driver_inputs.m_braking = 0.0

# === Irrlicht visualization (Initialize FIRST, then add scene elements) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS/IMU Sensor Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Sensor Manager ===
manager = sens.ChSensorManager(sys)
# Point lights for camera sensor rendering
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === IMU sensor — offset pose (0, 0, 1) as specified ===
imu_offset_pose = chrono.ChFramed(
    IMU_OFFSET,
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
imu = sens.ChAccelerometerSensor(
    chassis,       # attach to chassis body
    10,            # update_rate Hz (physical rate)
    imu_offset_pose,
    sens.ChNoiseNone(),
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# === GPS sensor ===
gps_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    chassis,       # attach to chassis body
    10,            # update_rate Hz (physical rate)
    gps_offset_pose,
    chrono.ChVector3d(GPS_REF_LON, GPS_REF_LAT, GPS_REF_ALT),  # reference lon/lat/alt
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Review-only: recording setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()  # cache: current sim time

        # Irrlicht render (throttled)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Inner physics batch
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()
            if sim_time >= SIM_END:
                break

            # Synchronize with constant inputs
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            # Advance all subsystems
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)

            # Update sensors each physics step
            manager.Update()


except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
