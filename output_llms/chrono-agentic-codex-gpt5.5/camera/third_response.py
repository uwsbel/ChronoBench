"""Camera sensor mounted on a textured rigid box in a PyChrono NSC system.

The scene replaces an external mesh target with a single ChBodyEasyBox, attaches
an RGB camera sensor to that real box body, and records the sensor view from a
rear-above offset pose. The box is fixed so the requested camera attachment and
offset remain easy to inspect while the Irrlicht view provides an external
view of the same setup.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants ===
SIDE = 2.0
BOX_DENSITY = 1000.0
FRICTION = 0.7
RESTITUTION = 0.1
TIME_STEP = 1.0e-3
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
CAMERA_UPDATE_RATE = 30.0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FOV = 1.408
CAMERA_OFFSET = chrono.ChVector3d(-7.0, 0.0, 3.0)
CAMERA_TILT = 0.2


# === System & Materials ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.GetSolver().AsIterative().SetMaxIterations(80)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(FRICTION)
contact_mat.SetRestitution(RESTITUTION)


# === Bodies ===
box = chrono.ChBodyEasyBox(SIDE, SIDE, SIDE, BOX_DENSITY, True, True, contact_mat)
box.SetPos(chrono.ChVector3d(0.0, 1.0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

floor = chrono.ChBodyEasyBox(12.0, 0.2, 12.0, 1000.0, True, True, contact_mat)
floor.SetPos(chrono.ChVector3d(0.0, -0.1, 0.0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box_body = box  # cache: camera and logging reuse the same body handle


# === Sensor Camera ===
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-9.0, 3.0, 20.0),
    chrono.ChColor(0.8, 0.8, 0.8),
    250.0,
)

offset_pose = chrono.ChFramed(
    CAMERA_OFFSET,
    chrono.QuatFromAngleAxis(CAMERA_TILT, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
camera = sens.ChCameraSensor(
    box_body,
    CAMERA_UPDATE_RATE,
    offset_pose,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
)
camera.SetName("Box Mounted Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "Box Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(camera)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Box-mounted camera sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-9.0, 4.0, 6.0), chrono.ChVector3d(0.0, 1.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    1.0,
    1.0,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.Q_ROTATE_Y_TO_Z),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main Loop ===
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame += 1

        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: reused for log row and stop check
            box_pos = box_body.GetPos()  # cache: reused for all logged coordinates
            manager.Update()
            rgba_buffer = camera.GetMostRecentRGBA8Buffer()
            if rgba_buffer.HasData():  # guard: buffer is empty before the first camera tick
                _ = rgba_buffer.GetRGBA8Data()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:  # disk or permission error from saved sensor frames
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # Chrono solver or sensor pipeline failure
    traceback.print_exc()
    raise
finally:
    pass
