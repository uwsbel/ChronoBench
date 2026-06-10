"""
Curiosity Mars Rover simulation with a chassis-mounted Lidar sensor.

System type : ChSystemNSC (non-smooth contact, NSC contact materials)
Robot       : robot.Curiosity driven by CuriosityDCMotorControl
Terrain     : rigid ground plane (ChBodyEasyBox, NSC contact material)
Sensor      : sens.ChLidarSensor mounted on the rover chassis, producing a
              rotating 3-D point cloud via ChFilterVisualize and ChFilterPCfromDepth
Behavior    : rover drives forward (default DC-motor) with a gentle steering ramp;
              the lidar scans the surroundings and displays a live point-cloud window.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens

# === Constants ===
TIME_STEP   = 1e-3          # physics step size (s)
SIM_END     = 20.0          # simulation end time (s)
RENDER_FPS  = 50.0          # Irrlicht review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# Rover spawn
ROVER_INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.2)   # slightly above ground surface
ROVER_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity quaternion (w,x,y,z)

# Steering profile parameters
STEER_START   = 3.0          # time to begin ramping steering
STEER_PEAK    = 8.0          # time steering reaches maximum
STEER_END     = 13.0         # time steering returns to zero
MAX_STEERING  = math.pi / 6  # ~30 deg maximum steering angle (rad)

# Lidar parameters
LIDAR_UPDATE_RATE   = 5.0        # Hz
LIDAR_H_SAMPLES     = 800        # horizontal sample count
LIDAR_V_SAMPLES     = 300        # vertical sample count
LIDAR_MAX_RANGE     = 100.0      # metres
LIDAR_HFOV          = 2 * chrono.CH_PI          # full 360° horizontal FOV
LIDAR_MAX_VERT      = chrono.CH_PI / 12         # +15° vertical
LIDAR_MIN_VERT      = -chrono.CH_PI / 6         # -30° vertical

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-up, gravity along -Z
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
# NSC contact material for wheel-ground contact
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

# Fixed flat ground; top surface at z = 0 for Curiosity spawn
ground = chrono.ChBodyEasyBox(40, 40, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))   # box top at z = 0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Curiosity rover ===
rover  = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)                                                  # must precede Initialize
rover.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

chassis = rover.GetChassis().GetBody()   # cache: fetched once, reused for sensor mount

# === Sensor manager ===
manager = sens.ChSensorManager(system)

# Point lights for the OptiX rendering pipeline (camera sensor only; lidar ignores them)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Lidar sensor (chassis-mounted) ===
# Offset pose: sit on top of the rover chassis, facing forward
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.5),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                              # body the sensor is attached to
    LIDAR_UPDATE_RATE,                    # update rate (Hz)
    lidar_offset,                         # mount pose relative to chassis
    LIDAR_H_SAMPLES,                      # horizontal sample count
    LIDAR_V_SAMPLES,                      # vertical sample count
    LIDAR_HFOV,                           # horizontal FOV (rad) — full 360°
    LIDAR_MAX_VERT,                       # max vertical angle
    LIDAR_MIN_VERT,                       # min vertical angle
    LIDAR_MAX_RANGE,                      # max range (m)
    sens.LidarBeamShape_RECTANGULAR,      # beam shape
    2,                                    # sample radius
    0.003,                                # vertical divergence angle
    0.003,                                # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)   # collection window = 1/rate

# Lidar filter chain (order matters):
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth → XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access to XYZI point cloud
manager.AddSensor(lidar)

# === Visualization (Irrlicht) ===
# Initialize FIRST, then add scene elements (Irrlicht call-order contract)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover with Lidar Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3.0, 2.0), chrono.ChVector3d(0, 0, 0.3))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only recording setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        t = system.GetChTime()

        # --- Steering profile (smooth ramp) ---
        if t < STEER_START:
            steering = 0.0
        elif t < STEER_PEAK:
            steering = MAX_STEERING * (t - STEER_START) / (STEER_PEAK - STEER_START)
        elif t < STEER_END:
            steering = MAX_STEERING * (STEER_END - t) / (STEER_END - STEER_PEAK)
        else:
            steering = 0.0

        driver.SetSteering(steering)
        rover.Update()   # propagate steering into rover motors

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            manager.Update()                  # update all sensors once per physics step
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad rover state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
