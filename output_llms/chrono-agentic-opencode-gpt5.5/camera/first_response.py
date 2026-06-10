"""Fixed OBJ mesh scene with an orbiting camera sensor.

This NSC PyChrono scene loads a Wavefront triangular mesh as a fixed body,
renders it in Irrlicht, and attaches a camera sensor to the mesh body. The
camera offset is updated around the mesh during the run, with RGB/noise and
grayscale filter streams saved and visualized. Each physics step prints guarded
camera buffer status so empty sensor frames are not read before the first tick.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === mesh, sensor, and timing values are precomputed once
TIME_STEP = 1.0e-3
SIM_END = 2.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
CAMERA_RATE = 30.0
ORBIT_RADIUS = 2.4
ORBIT_HEIGHT = 0.8
ORBIT_RATE = 0.75
IMAGE_W = 1280
IMAGE_H = 720
FOV = 1.408
MESH_FILE = "sensor/geometries/suzanne.obj"
MESH_TARGET_HEIGHT = 1.2


def orbit_pose(time):
    """Return the camera offset frame for the current orbit angle."""
    angle = ORBIT_RATE * time
    x = ORBIT_RADIUS * math.cos(angle)
    y = ORBIT_RADIUS * math.sin(angle)
    yaw = angle + math.pi
    pitch = math.atan2(-ORBIT_HEIGHT, ORBIT_RADIUS)
    rot = chrono.QuatFromAngleZ(yaw) * chrono.QuatFromAngleY(pitch)
    return chrono.ChFramed(chrono.ChVector3d(x, y, ORBIT_HEIGHT), rot)


# === System & Mesh Body === one fixed collidable Wavefront mesh for sensor rendering
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(50)

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

mesh_path = chrono.GetChronoDataFile(MESH_FILE)
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, True, True)
mesh_vertices = mesh.GetCoordsVertices()  # cache: measured once before scaling
min_x = min(v.x for v in mesh_vertices)
min_y = min(v.y for v in mesh_vertices)
min_z = min(v.z for v in mesh_vertices)
max_x = max(v.x for v in mesh_vertices)
max_y = max(v.y for v in mesh_vertices)
max_z = max(v.z for v in mesh_vertices)
mesh_raw_z = max_z - min_z
mesh_center_local = chrono.ChVector3d(
    0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.5 * (min_z + max_z)
)
mesh_scale = MESH_TARGET_HEIGHT / mesh_raw_z  # precomputed once
mesh.Transform(-mesh_center_local, chrono.ChMatrix33d(1.0))
mesh.Transform(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChMatrix33d(mesh_scale))
mesh_body = chrono.ChBodyEasyMesh(mesh, 1000.0, True, True, True, mat, 0.002)
mesh_body.SetName("fixed_suzanne_obj_mesh")
mesh_body.SetPos(chrono.ChVector3d(0.0, 0.0, MESH_TARGET_HEIGHT * 0.7))
mesh_body.SetFixed(True)
mesh_body.EnableCollision(True)
sys.AddBody(mesh_body)

ground = chrono.ChBodyEasyBox(5.0, 5.0, 0.1, 1000.0, True, True, mat)
ground.SetName("ground")
ground.SetPos(chrono.ChVector3d(0.0, 0.0, -0.08))
ground.SetFixed(True)
ground_vis_mat = chrono.ChVisualMaterial()
ground_vis_mat.SetDiffuseColor(chrono.ChColor(0.55, 0.55, 0.58))
ground_vis_mat.SetSpecularColor(chrono.ChColor(0.15, 0.15, 0.15))
ground_vis_mat.SetRoughness(0.7)
ground.GetVisualShape(0).AddMaterial(ground_vis_mat)
sys.AddBody(ground)

mesh_center = mesh_body.GetPos()  # cache: fixed body center reused by Irrlicht and logs
initial_eye = mesh_center + orbit_pose(0.0).GetPos()  # precomputed once


# === Sensor Manager & Camera === prompt-required camera with noise, visualization, and saves
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 5.0), chrono.ChColor(1.0, 1.0, 1.0), 500.0
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-3.0, -2.0, 4.0), chrono.ChColor(0.8, 0.8, 0.8), 300.0
)
manager.scene.AddAreaLight(
    chrono.ChVector3f(0.0, 0.0, 3.5),
    chrono.ChColor(0.7, 0.7, 0.7),
    300.0,
    chrono.ChVector3f(1.0, 0.0, 0.0),
    chrono.ChVector3f(0.0, 1.0, 0.0),
)

cam = sens.ChCameraSensor(mesh_body, CAMERA_RATE, orbit_pose(0.0), IMAGE_W, IMAGE_H, FOV)
cam.SetName("Orbiting Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(IMAGE_W, IMAGE_H, "RGB Camera"))
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
cam.PushFilter(sens.ChFilterGrayscale())
cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale Camera"))
cam.PushFilter(sens.ChFilterSave("cam/gray/"))
cam.PushFilter(sens.ChFilterR8Access())
manager.AddSensor(cam)


# === Visualization === Irrlicht window is separate from the sensor camera
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Fixed Mesh with Orbiting Camera Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(initial_eye, mesh_center)
vis.AddTypicalLights()
vis.AddGrid(
    0.25,
    0.25,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main Loop === update sensor orbit, print guarded buffers, and step physics
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = sys.GetChTime()
            current_pose = orbit_pose(time)
            current_pos = current_pose.GetPos()  # cache: reused for camera, print, and logging
            vis.UpdateCamera(mesh_center + current_pos, mesh_center)
            cam.SetOffsetPose(current_pose)
            manager.Update()

            rgba = cam.GetMostRecentRGBA8Buffer()
            gray = cam.GetMostRecentR8Buffer()
            rgba_has_data = rgba.HasData()
            gray_has_data = gray.HasData()
            print(
                f"time={time:.3f}, camera_offset=({current_pos.x:.3f}, "
                f"{current_pos.y:.3f}, {current_pos.z:.3f}), "
                f"rgba_has_data={rgba_has_data}, gray_has_data={gray_has_data}"
            )

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:  # file permissions / output directory failure
    print(f"file output error: {exc}")
    raise
except (RuntimeError, ValueError) as exc:  # sensor or solver runtime failure
    print(f"simulation runtime error: {exc}")
    raise
finally:
    print(f"finished at t={sys.GetChTime():.3f}")
