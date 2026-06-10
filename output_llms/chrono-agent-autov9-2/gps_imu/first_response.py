"""HMMWV with onboard IMU + GPS sensors on a flat rigid-terrain road.

Model
-----
A full-model HMMWV wheeled vehicle (SMC contact) drives forward on a flat
RigidTerrain patch under Irrlicht visualization. Three sensors ride on the
chassis body and are pumped by a ChSensorManager every physics step:

  * ChAccelerometerSensor  -> linear acceleration  [ax, ay, az]  (m/s^2)
  * ChGyroscopeSensor      -> angular velocity      [wx, wy, wz]  (rad/s)
  * ChGPSSensor            -> geodetic fix          [lon, lat, alt]  (deg/deg/m)

System type: ChSystemSMC (created and owned by the veh.HMMWV_Full wrapper).
Main bodies: chassis + four wheel/spindle assemblies + the terrain patch body.
Expected behavior: a scripted driver releases the brake and applies a steady
throttle, so the HMMWV accelerates down the road; the IMU then reports a clear
non-zero forward acceleration and yaw-rate signature while the GPS fix drifts
along the path. The vehicle mass is printed once at startup.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === geometry / physics / timing (no bare literals downstream)
time_step = 2.0e-3                  # integration step (s)
tire_step = 1.0e-3                  # tire model sub-step (s)
sim_end = 12.0                      # total simulated time (s)
render_fps = 50.0                   # review-video frame rate (Hz)
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

TERRAIN_LENGTH = 200.0             # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0              # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0                # flat road surface height (m)

SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis-origin height above wheel bottom (m)
TIRE_RADIUS = 0.46                 # nominal HMMWV tire radius for the footprint assert (m)
ZTOL = 0.10                        # allowed wheel-bottom clearance/overlap vs road (m)

INIT_X, INIT_Y = -80.0, 0.0        # spawn near the back of the patch, facing +X
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QUNIT

# GPS reference datum (lon, lat, alt) mapping world origin to geodetic coords.
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)
SENSOR_RATE = 100.0                # IMU/GPS update rate (Hz)
IMU_OFFSET = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)  # chassis frame


# === Scripted driver === time-based open-loop control (no human-in-the-loop)
class ScriptedDriver(veh.ChDriver):
    """Releases the brake after a settle phase, then holds steady throttle."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 1.0:                    # settle on the suspension first
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        self.SetSteering(0.0)             # straight-line run

# === Vehicle (HMMWV_Full wrapper) === wrapper creates + owns the ChSystemSMC
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so the HMMWV actually drives
hmmwv.SetTireStepSize(tire_step)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
vehicle = hmmwv.GetVehicle()               # cache: vehicle subsystem handle, reused below
# wheels/spindles: vehicle.GetAxle(i)...; terrain patch body created below
# joints: suspension + steering links created inside the wrapper

# Contact/collision IS present (tires vs terrain) -> Bullet collision is required.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

vehicle_mass = vehicle.GetMass()           # cache: queried once for the startup report
print(f"Vehicle mass: {vehicle_mass:.1f} kg")

# === Footprint check === assert wheels rest on the road, not through it
spindle_world = []
for axle in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} vs "
    f"road top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Terrain === flat rigid patch under the vehicle (Bullet contacts)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver === scripted open-loop schedule
driver = ScriptedDriver(vehicle)
driver.Initialize()

# === Sensors === IMU (accelerometer + gyroscope) + GPS on the chassis body
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))
no_noise = sens.ChNoiseNone()

acc_sensor = sens.ChAccelerometerSensor(chassis, SENSOR_RATE, IMU_OFFSET, no_noise)
acc_sensor.SetName("IMU_accelerometer")
acc_sensor.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc_sensor)

gyro_sensor = sens.ChGyroscopeSensor(chassis, SENSOR_RATE, IMU_OFFSET, no_noise)
gyro_sensor.SetName("IMU_gyroscope")
gyro_sensor.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro_sensor)

gps_sensor = sens.ChGPSSensor(chassis, SENSOR_RATE, IMU_OFFSET, GPS_REFERENCE, no_noise)
gps_sensor.SetName("GPS")
gps_sensor.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps_sensor)

# === Visualization === full vehicle-aware Irrlicht scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with IMU + GPS sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(INIT_X - 8.0, -8.0, 4.0), init_loc)
vis.AttachVehicle(vehicle)
vis.AttachDriver(driver)

# === Main loop === render-cadence outer loop; physics + sensors in inner batch


frame = 0
try:
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

            manager.Update()              # pump IMU + GPS every physics step

            driver.Advance(time_step)
            terrain.Advance(time_step)
            hmmwv.Advance(time_step)      # advances the wrapper-owned ChSystemSMC
            vis.Advance(time_step)


            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
