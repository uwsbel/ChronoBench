"""GPS + IMU sensing on a wheeled vehicle driving over flat rigid terrain.

Model
-----
An HMMWV full-vehicle wrapper (ChSystemNSC, owned internally by the wrapper) is
spawned on a flat RigidTerrain patch. A constant open-loop driver applies a fixed
steering of 0.6 and throttle of 0.5 for the whole run, so the vehicle accelerates
forward while curving — producing a smooth arc that the onboard sensors observe.

Sensors (the subject of this scene)
------------------------------------
An IMU (a ChAccelerometerSensor + a ChGyroscopeSensor) and a ChGPSSensor are rigidly
mounted on the chassis through a ChSensorManager. The IMU offset pose is
chrono.ChVector3d(0, 0, 1) (1 m above the chassis origin). The GPS reports geodetic
coordinates relative to a fixed reference; its buffer order is [lon, lat, alt, time].

Expected behavior
------------------
The chassis accelerates from rest and steers right, tracing a curved path; the
accelerometer registers forward/lateral acceleration, the gyroscope a steady yaw rate,
and the GPS latitude/longitude sweep out the corresponding ground-track arc.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / sensor configuration
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire substep (s)
SIM_END = 12.0                         # total simulated time (s)
RENDER_FPS = 50.0                      # review-video frame rate
SENSOR_UPDATE_RATE = 100.0            # IMU/GPS update rate (Hz)

TERRAIN_LENGTH = 200.0                 # rigid patch X extent (m)
TERRAIN_WIDTH = 200.0                  # rigid patch Y extent (m)
TERRAIN_HEIGHT = 0.0                   # patch top Z (m)

SUSPENSION_REF_HEIGHT = 0.5            # chassis-origin height above wheel-bottom at rest
INIT_X, INIT_Y = -30.0, 0.0            # spawn XY on the patch (well inside the footprint)
INIT_Z = TERRAIN_HEIGHT + SUSPENSION_REF_HEIGHT

CONST_STEERING = 0.6                   # prompt: constant steering for the whole run
CONST_THROTTLE = 0.5                   # prompt: constant throttle for the whole run

IMU_OFFSET = chrono.ChVector3d(0, 0, 1)            # prompt: IMU offset pose on the chassis
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)  # geodetic origin (lon, lat, alt)

# Derived constants (precomputed once — never recomputed in the loop)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Vehicle === HMMWV full wrapper owns and creates its ChSystemNSC + bodies/joints
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
vehicle.SetTireType(veh.TireModelType_TMEASY)   # slip-curve tire so the vehicle drives on rigid ground
vehicle.SetTireStepSize(TIRE_STEP)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = vehicle.GetSystem()                       # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()                 # cache: main chassis rigid body, reused every step
# wheels/spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints: suspension + steering links inside the wrapper

# Contact/collision IS present (vehicle wheels vs terrain) -> set the Bullet collision system.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Footprint check === assert the wheels rest on (not through) the terrain after Initialize
TIRE_RADIUS = 0.464                                # HMMWV tire radius (m), from wheel geometry
veh_obj = vehicle.GetVehicle()
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_HEIGHT - 0.10, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs "
    f"terrain top z={TERRAIN_HEIGHT:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === flat rigid patch under the vehicle (rigid contacts via Bullet)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver === constant open-loop steering + throttle for the whole run
driver_inputs = veh.DriverInputs()
driver_inputs.m_steering = CONST_STEERING          # constant steering 0.6
driver_inputs.m_throttle = CONST_THROTTLE          # constant throttle 0.5
driver_inputs.m_braking = 0.0

# === Sensors === IMU (accelerometer + gyroscope) + GPS rigidly mounted on the chassis
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100), chrono.ChColor(1, 1, 1), 5000.0)
noise_none = sens.ChNoiseNone()                     # cache: shared no-noise model reused by all sensors

imu_offset_pose = chrono.ChFramed(IMU_OFFSET, chrono.QUNIT)   # precomputed once
accelerometer = sens.ChAccelerometerSensor(chassis, SENSOR_UPDATE_RATE, imu_offset_pose, noise_none)
accelerometer.SetName("IMU - Accelerometer")
accelerometer.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(accelerometer)

gyroscope = sens.ChGyroscopeSensor(chassis, SENSOR_UPDATE_RATE, imu_offset_pose, noise_none)
gyroscope.SetName("IMU - Gyroscope")
gyroscope.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyroscope)

gps_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)   # precomputed once
gps = sens.ChGPSSensor(chassis, SENSOR_UPDATE_RATE, gps_offset_pose, GPS_REFERENCE, noise_none)
gps.SetName("GPS")
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("GPS + IMU on a wheeled vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 50, 50,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
vis.AttachVehicle(vehicle.GetVehicle())

# === Output setup ===

# === Main loop === render once per frame; advance the full vehicle stack + sensors per step
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = system.GetChTime()
            manager.Update()                # pump IMU + GPS every physics step
            driver_inputs.m_steering = CONST_STEERING
            driver_inputs.m_throttle = CONST_THROTTLE
            driver_inputs.m_braking = 0.0
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)      # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review videos + GPS trajectory plot, then clean frames
