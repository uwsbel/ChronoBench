"""Fixed OBJ mesh scene with an orbiting RGB camera sensor.

This PyChrono NSC scene loads a Wavefront OBJ triangular mesh as a fixed body,
adds an Irrlicht view for review, and attaches a camera sensor to the mesh body.
The camera offset is updated each step so it orbits the mesh while noise,
visualization, save, and access filters produce camera image buffers.
"""

import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === model scale, timing, and camera orbit are fixed for reproducible output
time_step = 1.0e-3
sim_end = 4.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
mesh_density = 1000.0
camera_rate = 30.0
camera_width = 1280
camera_height = 720
camera_fov = 1.408
orbit_radius = 4.0
orbit_height = 2.0
orbit_rate = 0.75
mesh_path = chrono.GetChronoDataFile("models/cube.obj")


def camera_orbit_pose(time):
    """Return a body-local camera pose that orbits and points at the fixed mesh."""
    angle = orbit_rate * time
    eye = chrono.ChVector3d(
        orbit_radius * math.cos(angle),
        orbit_radius * math.sin(angle),
        orbit_height,
    )
    yaw = math.atan2(-eye.y, -eye.x)
    horizontal_range = math.hypot(eye.x, eye.y)
    pitch = math.atan2(-eye.z, horizontal_range)
    rot = chrono.QuatFromAngleZ(yaw) * chrono.QuatFromAngleY(-pitch)
    return chrono.ChFramed(eye, rot)


# === System === fixed visual mesh with collision geometry for sensor visibility
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)


# === Mesh body === OBJ file is loaded as a fixed triangular mesh body
mesh_mat = chrono.ChContactMaterialNSC()
mesh_mat.SetFriction(0.7)
mesh_mat.SetRestitution(0.0)
mesh_body = chrono.ChBodyEasyMesh(mesh_path, mesh_density, True, True, True, mesh_mat)
mesh_body.SetName("fixed_wavefront_mesh")
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
sys.AddBody(mesh_body)
mesh_center = mesh_body.GetPos()  # cache: target reused by Irrlicht and sensor orbit


# === Sensor manager === camera sensor has lights, noise, visualization, save, and access filters
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(3, 3, 6),
    chrono.ChColor(1.0, 1.0, 1.0),
    80.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-4, -2, 5),
    chrono.ChColor(0.7, 0.7, 0.7),
    80.0,
)
manager.scene.AddAreaLight(
    chrono.ChVector3f(0, 0, 5),
    chrono.ChColor(0.6, 0.6, 0.6),
    60.0,
    chrono.ChVector3f(2, 0, 0),
    chrono.ChVector3f(0, 2, 0),
)

camera = sens.ChCameraSensor(
    mesh_body,
    camera_rate,
    camera_orbit_pose(0.0),
    camera_width,
    camera_height,
    camera_fov,
)
camera.SetName("Orbiting RGB Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
camera.PushFilter(sens.ChFilterVisualize(camera_width, camera_height, "Noisy RGB Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
camera.PushFilter(sens.ChFilterGrayscale())
camera.PushFilter(sens.ChFilterVisualize(640, 360, "Noisy Grayscale Camera"))
camera.PushFilter(sens.ChFilterSave("cam/gray/"))
manager.AddSensor(camera)


# === Visualization === Irrlicht review window is separate from the camera sensor output
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Fixed Wavefront Mesh with Orbiting Camera Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -5, 3), mesh_center)
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.02), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)

# === Main loop === update sensor pose, publish buffers, render, and step the fixed scene
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        sim_time = sys.GetChTime()  # cache: reused for orbit, logging, and prints
        offset_pose = camera_orbit_pose(sim_time)
        camera.SetOffsetPose(offset_pose)
        cam_pos = offset_pose.GetPos()
        review_eye = mesh_center + cam_pos
        vis.UpdateCamera(review_eye, mesh_center)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            manager.Update()
            buf = camera.GetMostRecentRGBA8Buffer()
            has_data = buf.HasData()
            if has_data:
                rgba = buf.GetRGBA8Data()
                print(
                    f"time={sys.GetChTime():.3f} camera_buffer shape={rgba.shape} "
                    f"first_pixel={rgba[0, 0].tolist()}"
                )
            else:
                print(f"time={sys.GetChTime():.3f} camera_buffer empty")
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:
    print(f"simulation failed during sensor update or dynamics: {exc}")
    raise
finally:
    pass
