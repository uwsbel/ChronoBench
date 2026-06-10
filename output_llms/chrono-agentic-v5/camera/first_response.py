"""Camera-sensor scene: a fixed triangular mesh sensed by an orbiting RGB camera.

Models a single static triangular mesh body (loaded from a Wavefront .obj) that is
NOT actuated and carries no collision geometry, so the physical system is a plain
ChSystemNSC with no contact. A ChSensorManager drives one ChCameraSensor whose
offset pose is rewritten every step to orbit the camera around the mesh. The camera
filter graph applies a constant-normal noise model plus live RGB and grayscale
visualizations, and the camera's RGBA8 host buffer is read and printed at each step.
Expected behavior: the camera sweeps a circular orbit around the stationary mesh and
emits a continuous stream of (noisy) RGB/grayscale frames; the scene is static.
"""

import math

import pychrono.core as chrono
import pychrono.sensor as sens

# === Parameters === geometry / camera / simulation constants (no bare literals downstream)
time_step = 1e-3                 # physics step (s)
sim_end = 20.0                   # simulation end time (s)
update_rate = 30                 # camera physical update rate (Hz) — NOT 1/dt
image_width = 1280               # camera image width (px)
image_height = 720               # camera image height (px)
fov = 1.408                      # camera horizontal field of view (rad)
mesh_scale = 2.0                 # uniform scale applied to the loaded mesh
orbit_radius = 10.0              # camera orbit radius around the mesh (m)
orbit_rate = 0.5                 # camera orbit angular rate (rad/s)
orbit_height = 1.0               # camera height above the mesh during the orbit (m)
mesh_file = "vehicle/hmmwv/hmmwv_chassis.obj"  # bundled Wavefront mesh asset

render_fps = 50.0                # render cadence (frames/s) for the throttled loop
render_every = max(1, round(1.0 / (render_fps * time_step)))    # precomputed once

# === System & gravity === plain NSC system; the mesh is fixed and has no contact
sys = chrono.ChSystemNSC()

# === Bodies === one fixed triangular-mesh body loaded from the .obj
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(mesh_file), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(mesh_scale))  # scale uniformly

trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("Triangular Mesh")
trimesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(True)        # static body — sensed, never moved
sys.Add(mesh_body)

# === Sensor manager & lighting === point + area lights illuminate the camera scene
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0,
                           chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

# === Camera sensor === attached to the mesh body; noise + RGB/gray visualize + save streams
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-orbit_radius, 0, orbit_height),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1)),
)
cam = sens.ChCameraSensor(mesh_body, update_rate, offset_pose, image_width, image_height, fov)
cam.SetName("Camera Sensor")
cam.SetLag(0)                    # truth: lag = 0
cam.SetCollectionWindow(0)      # camera exposure/collection window = 0

# filter chain (ORDER MATTERS — each Save snapshots the buffer AT its position)
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))           # constant-normal noise
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera"))  # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                               # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                            # SAVE stream #1: color PNGs
cam.PushFilter(sens.ChFilterGrayscale())                                 # convert to grayscale
cam.PushFilter(sens.ChFilterVisualize(image_width // 2, image_height // 2, "Grayscale Image"))  # gray preview
cam.PushFilter(sens.ChFilterSave("cam/gray/"))                           # SAVE stream #2: grayscale PNGs
cam.PushFilter(sens.ChFilterImageResize(image_width // 2, image_height // 2))  # downscale for downstream
cam.PushFilter(sens.ChFilterR8Access())                                  # host access to R8 buffer
manager.AddSensor(cam)          # push ALL filters BEFORE AddSensor

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
import pychrono.irrlicht as chronoirr
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Orbiting Camera Sensor on a Fixed Mesh")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-orbit_radius, 0, orbit_height + 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === orbit the camera, pump sensors every step, print the RGBA8 buffer

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            ch_time = sys.GetChTime()
            # orbit the camera around the mesh by rewriting its offset pose
            cam_x = -orbit_radius * math.cos(ch_time * orbit_rate)
            cam_y = -orbit_radius * math.sin(ch_time * orbit_rate)
            cam.SetOffsetPose(chrono.ChFramed(
                chrono.ChVector3d(cam_x, cam_y, orbit_height),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

            # read & print the camera RGBA8 host buffer (guard: empty before first tick)
            rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
            if rgba8_buffer.HasData():
                rgba8_data = rgba8_buffer.GetRGBA8Data()
                print("RGBA8 buffer received from cam. Camera resolution: {0}x{1}".format(
                    rgba8_buffer.Width, rgba8_buffer.Height))
                print("First Pixel: {0}".format(rgba8_data[0, 0, :]))

            manager.Update()            # pump sensors EVERY physics step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review + sensor videos, plot, drop frame dirs
