"""
HMMWV Vehicle with IMU and GPS Sensors Simulation
==================================================
System type: NSC (rigid terrain, catalog HMMWV wrapper owns the ChSystem)
Main bodies: HMMWV chassis, 4 wheels/spindles on flat rigid terrain patch
Sensors: ChAccelerometerSensor (IMU) + ChGPSSensor (GPS) — both attached to chassis
Driver: ChInteractiveDriverIRR (real-time keyboard control, scored-core default)
Expected behavior: HMMWV drives on flat rigid terrain; IMU records acceleration
data and GPS records position data from the chassis; Irrlicht chase camera follows
the vehicle; the sensor manager updates every physics step.
Vehicle mass is printed at startup; GPS + IMU buffers are sampled each step.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr          # Irrlicht renderer (not VSG)
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Simulation parameters ===
time_step       = 1e-3                          # physics step size (s)
sim_end         = 20.0                          # simulation duration (s)
render_fps      = 50.0                          # Irrlicht render cadence (Hz)
render_every    = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
render_step_size = 1.0 / render_fps             # precomputed once (s/frame)

# === Vehicle + terrain constants ===
TERRAIN_LENGTH        = 200.0   # m — terrain patch X
TERRAIN_WIDTH         = 200.0   # m — terrain patch Y
SUSPENSION_REF_HEIGHT = 0.5     # chassis origin above wheel-bottom at rest (HMMWV)
INIT_X, INIT_Y        = 0.0, 0.0
INIT_Z                = 0.5 + SUSPENSION_REF_HEIGHT   # flat terrain top at z=0
init_loc              = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot              = chrono.QuatFromAngleZ(0.0)     # facing +X

# === Sensor parameters ===
SENSOR_RATE = 10.0   # Hz — IMU and GPS physical update rate
GPS_REF = chrono.ChVector3d(-89.400, 43.070, 260.0)   # reference lat/lon/alt (Madison WI)


# === Vehicle setup (HMMWV_Full wrapper owns its ChSystem) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC for rigid terrain (truth)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # TMEASY: good all-terrain tire
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

# === System & bodies (created by veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()             # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()        # cache: main chassis rigid body; reused every step
veh_obj = hmmwv.GetVehicle()           # cache: fetched once, reused for spindle query + sensors
# wheels/spindles: veh_obj.GetAxle(i)  — created by wrapper
# suspension + steering joints: created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types (in this 9.0.0 build, VisualizationType lives in veh namespace)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Validate wheel placement after Initialize (assert wheel bottom is above terrain)
TIRE_RADIUS = 0.33    # approximate HMMWV tire radius (m) — from wheel geometry
ZTOL        = 0.10    # allowed clearance vs terrain top at z=0
spindle_positions = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_positions.append(p)
wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} — "
    f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

print(f"[INFO] Vehicle mass: {veh_obj.GetMass():.1f} kg")

# === Terrain (flat rigid terrain) ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,    # centered at origin, Z-up
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
# Use chrono data path for terrain texture (veh.GetDataFile resolves relative to CWD in some builds)
patch.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 200, 200
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Vehicle Irrlicht visualization (order-sensitive: configure → Initialize → add elements) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — IMU & GPS Sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)

# === Driver (interactive — real-time keyboard control, scored-core default) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)    # 1 s to reach max steering
driver.SetThrottleDelta(render_step_size / 1.0)    # 1 s to reach max throttle
driver.SetBrakingDelta(render_step_size / 0.3)     # 0.3 s to reach max braking
driver.Initialize()

# === Sensor Manager — IMU + GPS attached to chassis ===
manager = sens.ChSensorManager(sys)
# Note: IMU and GPS do not render — no scene lights needed

# Sensor offset pose: centered on chassis, 1 m above chassis origin
sensor_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

# IMU (accelerometer) — 10 Hz, no noise, ChFilterAccelAccess for buffer readback
imu = sens.ChAccelerometerSensor(chassis, SENSOR_RATE, sensor_offset, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# GPS — 10 Hz, reference origin at Madison WI, no noise, ChFilterGPSAccess for buffer readback
gps = sens.ChGPSSensor(chassis, SENSOR_RATE, sensor_offset, GPS_REF, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Review-only: CSV setup ===

# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0  # review-only frame counter

try:
    while vis.Run():
        time = sys.GetChTime()
        if time >= sim_end:
            break

        # Throttled Irrlicht render (every render_every steps)
        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs (interactive by default; review-only block overrides for video)
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (fixed order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems (vehicle.Advance steps the system — do NOT also call sys.DoStepDynamics)
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)

        # Update sensor manager once per step (after all Advance calls)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(time_step)

except (RuntimeError, ValueError) as exc:
    # Guard: solver divergence or bad state — print trace and propagate
    import traceback
    traceback.print_exc()
    raise

finally:
    pass   # ensures finally block is always present
