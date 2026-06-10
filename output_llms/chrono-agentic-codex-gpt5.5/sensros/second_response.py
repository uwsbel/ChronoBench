"""2D lidar sensor bridge with ROS laser-scan output.

This PyChrono NSC simulation rotates a sensor carrier in a simple target field.
A 2D lidar is attached to the carrier, uses named sensor filters, and is
registered with ChROS to publish LaserScan data on ~/output/lidar2d/data/scan.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants ===
TIME_STEP = 0.005
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
LIDAR_RATE = 5.0


def sensor_pose(x, y, z):
    """Return the carrier-frame 2D lidar mounting pose."""
    return chrono.ChFramed(chrono.ChVector3d(x, y, z), chrono.QUNIT)


def buffer_has_data(buffer):
    """Guard ROS publication until the lidar buffer has data."""
    return bool(buffer and buffer.HasData())


# === System & Bodies ===
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
material = chrono.ChContactMaterialNSC()

base_link = chrono.ChBody()
base_link.SetName("base_link")
base_link.SetFixed(True)
system.AddBody(base_link)

carrier = chrono.ChBodyEasyBox(0.9, 0.35, 0.25, 1000.0, True, False, material)
carrier.SetName("sensor_carrier")
carrier.SetPos(chrono.ChVector3d(0.0, 0.0, 1.0))
carrier.SetFixed(True)
system.Add(carrier)
carrier_body = carrier  # cache: reused by the sensor and ROS handlers

for i, (x, y) in enumerate([(4.0, 0.0), (-3.0, 2.0), (1.5, -3.5)]):
    target = chrono.ChBodyEasyBox(0.35, 0.35, 1.0, 1000.0, True, False, material)
    target.SetName(f"lidar_target_{i}")
    target.SetFixed(True)
    target.SetPos(chrono.ChVector3d(x, y, 1.0))
    system.Add(target)

orbit_marker = chrono.ChBodyEasySphere(0.18, 1000.0, True, False, material)
orbit_marker.SetName("lidar_sweep_marker")
orbit_marker.SetPos(chrono.ChVector3d(2.0, 0.0, 1.0))
system.Add(orbit_marker)
marker_body = orbit_marker  # cache: animated visual reference for review


# === Sensor Manager & 2D Lidar ===
manager = sens.ChSensorManager(system)

lidar2d = sens.ChLidarSensor(
    carrier_body,
    LIDAR_RATE,
    sensor_pose(0.0, 0.0, 0.20),
    720,
    1,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    60.0,
    sens.LidarBeamShape_RECTANGULAR,
    1,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar2d.SetName("2D Lidar Sensor")
lidar2d.SetLag(0)
lidar2d.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar2d.PushFilter(sens.ChFilterDIAccess("2D Lidar Depth-Intensity Access"))
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterXYZIAccess("2D Lidar Point-Cloud Access"))
manager.AddSensor(lidar2d)


# === ROS Bridge ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25.0, carrier_body, "~/output/body/data"))
tf_handler = chros.ChROSTFHandler(25.0)
tf_handler.AddTransform(base_link, base_link.GetName(), carrier_body, carrier_body.GetName())
ros_manager.RegisterHandler(tf_handler)
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(
        LIDAR_RATE,
        lidar2d,
        "~/output/lidar2d/data/scan",
        chros.ChROSLidarHandlerMessageType_LASER_SCAN,
    )
)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("sensros 2D lidar ROS bridge")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6.0, -6.0, 3.0), chrono.ChVector3d(0.0, 0.0, 1.0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 12, 12, chrono.ChCoordsysd(), chrono.ChColor(0.35, 0.35, 0.35))

ros_manager.Initialize()


# === Main Loop ===
frame = 0

try:
        while vis.Run() and system.GetChTime() < SIM_END:
            render_time = system.GetChTime()
            carrier_body.SetRot(chrono.QuatFromAngleZ(0.4 * render_time))
            marker_body.SetPos(chrono.ChVector3d(2.0 * math.cos(1.2 * render_time), 2.0 * math.sin(1.2 * render_time), 1.0))
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()  # cache: reused by ROS and logging this step
                pos = carrier_body.GetPos()  # cache: reused by review logging
                manager.Update()
                if buffer_has_data(lidar2d.GetMostRecentDIBuffer()):
                    if not ros_manager.Update(time, TIME_STEP):
                        break
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
except (OSError, IOError) as exc:
    print(f"File output failed: {exc}")
    raise
except (RuntimeError, ValueError) as exc:
    print(f"Simulation failed: {exc}")
    raise
finally:
    pass
