"""HMMWV wheeled-vehicle GPS + IMU sensing demo (NSC, rigid terrain).

Models a full HMMWV on a flat rigid-terrain patch driven with a constant
open-loop maneuver (steering 0.6, throttle 0.5), so the chassis follows a
steady curving trajectory. A ChSensorManager carries two onboard sensors that
ride the chassis body:
  * a GPS sensor (referenced to a lat/lon/alt origin) reporting geodetic
    position along the path,
  * an IMU accelerometer reporting chassis proper acceleration,
with the IMU offset pose placed at (0, 0, 1) in the chassis frame.
Expected behavior: the vehicle accelerates and turns; the GPS trace sweeps a
curved geodetic path and the accelerometer registers the lateral/longitudinal
acceleration of the maneuver. An Irrlicht window provides the review view.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Parameters === geometry / physics / sensor constants (no bare literals downstream)
time_step = 1e-3
sim_end = 12.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

CONST_STEERING = 0.6        # prompt: constant steering held all sim
CONST_THROTTLE = 0.5        # prompt: constant throttle held all sim

INIT_LOC = chrono.ChVector3d(0, 0, 0.5)        # chassis spawn over the terrain patch
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)   # (lon, lat, alt) origin
IMU_OFFSET = chrono.ChVector3d(0, 0, 1)        # prompt: IMU offset pose (0, 0, 1)
SENSOR_RATE = 10                               # GPS/IMU physical update rate (Hz)


# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV on rigid terrain (wrapper owns the ChSystemNSC)
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # rigid terrain -> NSC
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)          # rigid-terrain tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = vehicle.GetSystem()                  # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED, after Initialize
chassis = vehicle.GetChassisBody()         # cache: main chassis rigid body, reused below
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain === flat rigid patch supporting the wheels
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         200.0, 200.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# === Visualization === full Irrlicht vehicle scene: window + sky + chase cam + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS + IMU sensing")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                  # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === constant open-loop inputs (steering 0.6, throttle 0.5)
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_every * time_step / 1.0)
driver.SetThrottleDelta(render_every * time_step / 1.0)
driver.SetBrakingDelta(render_every * time_step / 0.3)
driver.Initialize()

# === Sensors === GPS + IMU riding the chassis (no scene lights — neither renders)
manager = sens.ChSensorManager(sys)

imu_offset_pose = chrono.ChFramed(IMU_OFFSET, chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
imu = sens.ChAccelerometerSensor(chassis, SENSOR_RATE, imu_offset_pose, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

gps_offset_pose = chrono.ChFramed(IMU_OFFSET, chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
gps = sens.ChGPSSensor(chassis, SENSOR_RATE, gps_offset_pose, GPS_REFERENCE, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Main loop === advance vehicle + sensors, hold constant driver inputs
render_step_size = render_every * time_step
realtime_timer = chrono.ChRealtimeStepTimer()


frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    time = sys.GetChTime()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        time = sys.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)
        manager.Update()                # pump sensors every physics step


        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break

    realtime_timer.Spin(render_step_size)
