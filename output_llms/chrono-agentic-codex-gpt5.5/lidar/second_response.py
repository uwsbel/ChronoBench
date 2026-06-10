"""Box-mounted lidar scene using PyChrono NSC contact and Chrono::Sensor.

The simulation replaces a mesh host with a textured box and attaches two lidar
sensors to that box: a full 3D lidar and a planar 2D lidar with one vertical
channel. Fixed collision-enabled target objects surround the host so both lidars
produce visible depth and point-cloud returns while the Irrlicht window records a
review view.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants: choose stable dimensions for the host and scan targets ===
TIME_STEP = 1.0e-3
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
LIDAR_RATE = 5.0

SIDE = 1.0
BOX_DENSITY = 1000.0
GROUND_SIZE_X = 10.0
GROUND_SIZE_Y = 10.0
GROUND_SIZE_Z = 0.12
TARGET_SIZE = 0.6

HOST_POS = chrono.ChVector3d(0.0, 0.0, SIDE * 0.5)
GROUND_POS = chrono.ChVector3d(0.0, 0.0, -GROUND_SIZE_Z * 0.5)
LIDAR_OFFSET = chrono.ChVector3d(0.0, 0.0, SIDE * 0.75)
LIDAR_ROT = chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0))


# === System & materials: NSC rigid scene with Bullet collision for sensor geometry ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.7)
contact_mat.SetRestitution(0.0)


# === Bodies: fixed textured box host plus nearby scan targets ===
ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_SIZE_Z, BOX_DENSITY, True, True, contact_mat
)
ground.SetFixed(True)
ground.SetPos(GROUND_POS)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

host_box = chrono.ChBodyEasyBox(SIDE, SIDE, SIDE, BOX_DENSITY, True, True, contact_mat)
host_box.SetFixed(True)
host_box.SetPos(HOST_POS)
host_box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
lidar_housing = chrono.ChVisualShapeCylinder(0.16, 0.16)
host_box.AddVisualShape(lidar_housing, chrono.ChFramed(LIDAR_OFFSET, chrono.QUNIT))
sys.Add(host_box)

target_positions = [
    chrono.ChVector3d(2.4, 0.0, TARGET_SIZE * 0.5),
    chrono.ChVector3d(-2.4, 1.2, TARGET_SIZE * 0.5),
    chrono.ChVector3d(0.0, 2.8, TARGET_SIZE * 0.5),
    chrono.ChVector3d(1.8, -2.0, TARGET_SIZE * 0.5),
]
scan_targets = []
for idx, pos in enumerate(target_positions):
    target = chrono.ChBodyEasyBox(
        TARGET_SIZE, TARGET_SIZE, TARGET_SIZE, BOX_DENSITY, True, True, contact_mat
    )
    target.SetFixed(True)
    target.SetPos(pos)
    target.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/redwhite.png"))
    sys.Add(target)
    scan_targets.append(target)

host = host_box  # cache: sensor attachment body fetched once and reused


# === Sensor manager: lidar sensors attach directly to the box host ===
manager = sens.ChSensorManager(sys)

lidar_pose = chrono.ChFramed(LIDAR_OFFSET, LIDAR_ROT)
lidar_3d = sens.ChLidarSensor(
    host,
    LIDAR_RATE,
    lidar_pose,
    800,
    300,
    2.0 * chrono.CH_PI,
    chrono.CH_PI / 12.0,
    -chrono.CH_PI / 6.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_3d.SetName("Box Mounted 3D Lidar")
lidar_3d.SetLag(0)
lidar_3d.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar_3d.PushFilter(sens.ChFilterVisualize(800, 300, "3D Lidar Depth"))
lidar_3d.PushFilter(sens.ChFilterDIAccess())
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_3d)

lidar_2d = sens.ChLidarSensor(
    host,
    LIDAR_RATE,
    lidar_pose,
    800,
    1,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("Box Mounted 2D Lidar")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)


# === Visualization: Irrlicht review window with camera and ground reference ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Box Lidar Sensors")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, -6.0, 3.0), chrono.ChVector3d(0.0, 0.0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(
    1.0,
    1.0,
    12,
    12,
    chrono.ChCoordsysd(),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop: sensor updates and rigid-body stepping stay in the scored core ===
frame = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            manager.Update()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # sensor render or solver state failure
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
