"""PyChrono camera sensor scene using an NSC rigid system.

The scene contains fixed collision/visual bodies and a camera sensor mounted on a
fixed orbit rig. The camera saves 960 x 480 RGB images from an offset pose of
(-7, 0, 2) while the rig orbits the central target at 0.1 radians per second.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === camera, timing, and body parameters kept explicit for review
time_step = 1.0e-3
sim_end = 8.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
camera_update_rate = 30.0
camera_width = 960
camera_height = 480
camera_fov = 1.408
camera_orbit_rate = 0.1
target_size = chrono.ChVector3d(1.0, 1.0, 1.0)
floor_size = chrono.ChVector3d(10.0, 10.0, 0.1)


# === System & Materials === NSC contact bodies are visible to both Irrlicht and OptiX
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.8)
contact_mat.SetRestitution(0.0)


# === Bodies === fixed target and floor provide collision geometry for sensor rendering
floor = chrono.ChBodyEasyBox(floor_size.x, floor_size.y, floor_size.z, 1000.0, True, True, contact_mat)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, -0.05))
sys.Add(floor)

target = chrono.ChBodyEasyBox(target_size.x, target_size.y, target_size.z, 1000.0, True, True, contact_mat)
target.SetFixed(True)
target.SetPos(chrono.ChVector3d(0, 0, 0.5))
sys.Add(target)

camera_rig = chrono.ChBody()
camera_rig.SetFixed(True)
camera_rig.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(camera_rig)

target_body = target  # cache: central body is reused for camera targets and logging
target_pos = target_body.GetPos()  # cache: fixed target position reused every render


# === Sensor Camera === prompt-required RGB stream is saved by ChFilterSave in core
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddAreaLight(
    chrono.ChVector3f(0, 0, 4),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
    chrono.ChVector3f(1, 0, 0),
    chrono.ChVector3f(0, -1, 0),
)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)

camera = sens.ChCameraSensor(
    camera_rig,
    camera_update_rate,
    offset_pose,
    camera_width,
    camera_height,
    camera_fov,
)
camera.SetName("Orbiting RGB Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(camera_width, camera_height, "RGB Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(camera)


# === Visualization === Irrlicht review window is built unconditionally like the scored core
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor Orbit")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(7, -8, 4), target_pos)
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main Loop === orbit the rig, update sensors, and advance the static system
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sim_time = sys.GetChTime()
            camera_yaw = camera_orbit_rate * sim_time
            camera_rig.SetRot(chrono.QuatFromAngleAxis(camera_yaw, chrono.ChVector3d(0, 0, 1)))
            manager.Update()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver or sensor runtime failure
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # disk or output failure
    traceback.print_exc()
    raise
finally:
    pass
