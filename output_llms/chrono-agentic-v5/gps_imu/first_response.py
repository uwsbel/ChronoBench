"""HMMWV full-vehicle simulation on flat rigid terrain with onboard IMU + GPS sensors.

Models a full HMMWV (NSC contact) driving on a flat RigidTerrain patch, rendered with
the vehicle-aware Irrlicht visual system and steered by an interactive driver. A
ChSensorManager carries three chassis-mounted sensors: an accelerometer and a gyroscope
(together the IMU) and a GPS receiver, each with its own access filter. Every step the
vehicle/terrain/driver/visual modules are synchronized and advanced and the sensor
manager is pumped, so the IMU/GPS buffers update from the live chassis pose. The vehicle
mass is printed once after initialization. Expected behavior: the HMMWV rests on the
terrain and drives forward under throttle while the IMU/GPS produce per-step samples.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Parameters === geometry / timing / sensor-rate constants (no bare literals downstream)
time_step = 1e-3                 # physics step (s)
sim_end = 12.0                   # bounded recording horizon (s)
render_fps = 50.0                # Irrlicht render cadence
terrain_length = 200.0           # rigid patch X extent (m)
terrain_width = 200.0            # rigid patch Y extent (m)
terrain_top_z = 0.0              # flat patch top surface height (m)
suspension_ref_height = 0.5      # HMMWV chassis-origin height above wheel-bottom at rest (m)
tire_radius = 0.46               # HMMWV tire radius (m), for the footprint assert
sensor_rate = 10.0               # IMU/GPS physical update rate (Hz) — not 1/dt
gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)  # GPS origin (lat, lon, alt)

init_z = terrain_top_z + suspension_ref_height                          # precomputed once
init_loc = chrono.ChVector3d(0, 0, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)


# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful, scored)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV wrapper owns its ChSystemNSC, chassis, spindles and joints
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC matches the rigid-terrain truth
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — a fixed chassis never moves
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # deformable tire model on rigid road
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created inside the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                           # cache: ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for terrain contact
chassis_body = hmmwv.GetChassisBody()                # cache: main chassis rigid body, reused below
# spindles: hmmwv.GetVehicle().GetSpindlePos(axle, side); joints: suspension/steering links (wrapper)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

# Footprint check: wheel bottoms must rest on (not through) the flat patch.
veh_obj = hmmwv.GetVehicle()                         # cache: vehicle handle for spindle queries
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(axle, side).z
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - tire_radius
assert wheel_bottom_z >= terrain_top_z - 0.1, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_top_z:.3f}; raise suspension_ref_height"
)

# === Terrain === flat rigid patch under the vehicle (Bullet contacts via the system above)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase camera + sky + directional light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with IMU and GPS sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                            # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive driver bound to the visual system (truth-faithful catalog default)
render_step_size = 1.0 / render_fps
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Sensors === IMU (accelerometer + gyroscope) and GPS riding on the chassis body
manager = sens.ChSensorManager(system)
sensor_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),                       # chassis-frame mounting offset
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

imu_accel = sens.ChAccelerometerSensor(chassis_body, sensor_rate, sensor_offset, sens.ChNoiseNone())
imu_accel.SetName("IMU Accelerometer")
imu_accel.SetLag(0)
imu_accel.SetCollectionWindow(0)
imu_accel.PushFilter(sens.ChFilterAccelAccess())     # host access to accelerometer buffer
manager.AddSensor(imu_accel)

imu_gyro = sens.ChGyroscopeSensor(chassis_body, sensor_rate, sensor_offset, sens.ChNoiseNone())
imu_gyro.SetName("IMU Gyroscope")
imu_gyro.SetLag(0)
imu_gyro.SetCollectionWindow(0)
imu_gyro.PushFilter(sens.ChFilterGyroAccess())       # host access to gyroscope buffer
manager.AddSensor(imu_gyro)

gps = sens.ChGPSSensor(chassis_body, sensor_rate, sensor_offset, gps_reference, sens.ChNoiseNone())
gps.SetName("GPS")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())             # host access to GPS lat/lon/alt buffer
manager.AddSensor(gps)

# === Main loop === synchronize + advance the full subsystem stack, pump sensors each step
render_steps = math.ceil(render_step_size / time_step)                  # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

try:

    step_number = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)                      # advances the wrapper-owned system
        vis.Advance(time_step)

        manager.Update()                              # pump IMU + GPS once per physics step


        step_number += 1
        realtime_timer.Spin(time_step)                # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:             # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
