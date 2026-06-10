import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# ---------------------------------------------------------------------------
# Camera sensor parameters (physical units, mirror the 9.0.0 ground truth)
update_rate = 30.0          # camera update rate [Hz] — physical, not 1/dt
image_width = 1280          # horizontal pixel resolution
image_height = 720          # vertical pixel resolution
fov = 1.408                 # camera horizontal field of view [rad]
lag = 0.0                   # sensor signal lag [s]
exposure_time = 0.0         # camera collection/exposure window [s]
noise_model = "CONST_NORMAL"   # constant-normal additive image noise
vis_filter = True           # apply ChFilterVisualize previews
save_data = True            # apply ChFilterSave (writes PNGs)

# ---------------------------------------------------------------------------
# Simulation / orbit parameters
time_step = 1e-3            # integration step [s]
sim_end = 4.0              # total simulated time [s]
orbit_radius = 3.0         # camera orbit radius about the mesh [m]
orbit_rate = 0.5           # camera orbit angular rate [rad/s]
orbit_height = 2.0         # camera height above the mesh [m]
render_fps = 30.0          # Irrlicht review-frame cadence [fps]

# ---------------------------------------------------------------------------
# Physical system (Z-up). Sensor scenes use NSC with no contact between bodies,
# but the mesh body carries collision geometry so the sensor (OptiX) can see it.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # gravity off — static subject
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # collision system for the mesh body

# ---------------------------------------------------------------------------
# Load the triangular mesh from a Wavefront .obj and wrap it in a fixed body.
mesh_file = chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj")   # bundled triangular mesh
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_file, False, True)                       # load tris (no normals merge, with materials)
mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1.0))  # identity placement / unit scale

mesh_shape = chrono.ChVisualShapeTriangleMesh()                     # visual asset for the mesh
mesh_shape.SetMesh(mesh)
mesh_shape.SetName("Triangular Mesh")
mesh_shape.SetMutable(False)                                        # static geometry — immutable

mesh_body = chrono.ChBody()                                         # fixed body carrying the mesh
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                        # mesh at the world origin
mesh_body.AddVisualShape(mesh_shape, chrono.ChFramed())            # attach mesh visual at body frame
mesh_body.SetFixed(True)                                            # the mesh is a fixed scene body
sys.Add(mesh_body)

# ---------------------------------------------------------------------------
# Sensor manager + scene lighting (camera-only: point lights + one area light).
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddAreaLight(                                          # broad area light over the mesh
    chrono.ChVector3f(0, 0, 4),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
    chrono.ChVector3f(1, 0, 0),
    chrono.ChVector3f(0, -1, 0),
)

# ---------------------------------------------------------------------------
# Camera sensor attached to the mesh body. The offset pose is updated every
# step to orbit the camera around the mesh.
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-orbit_radius, 0, orbit_height),               # initial offset: behind + above the mesh
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),       # slight downward tilt about Y
)
cam = sens.ChCameraSensor(
    mesh_body,                                                       # body the camera rides on
    update_rate,                                                     # update rate [Hz]
    offset_pose,                                                     # offset pose on the body
    image_width,                                                     # image width [px]
    image_height,                                                    # image height [px]
    fov,                                                             # horizontal FOV [rad]
)
cam.SetName("Camera Sensor")
cam.SetLag(lag)                                                      # signal lag = 0
cam.SetCollectionWindow(exposure_time)                              # exposure/collection window = 0

# --- filter chain (ORDER MATTERS: each Save snapshots the buffer at its slot) ---
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))      # additive constant-normal image noise
if vis_filter:
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera"))   # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                          # host access to the RGBA8 buffer
if save_data:
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                   # SAVE stream #1: color PNGs
cam.PushFilter(sens.ChFilterGrayscale())                            # convert to grayscale
if vis_filter:
    cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale Camera"))   # grayscale preview
if save_data:
    cam.PushFilter(sens.ChFilterSave("cam/gray/"))                  # SAVE stream #2: grayscale PNGs
cam.PushFilter(sens.ChFilterImageResize(640, 360))                 # downsize for downstream consumers
cam.PushFilter(sens.ChFilterR8Access())                            # host access to the R8 (grayscale) buffer
manager.AddSensor(cam)                                              # register sensor AFTER all filters

# ---------------------------------------------------------------------------
# Irrlicht review window (separate from the sensor camera). Initialize first,
# then add scene elements (NO grid).
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                  # Z-up window camera
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor on Triangular Mesh")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, -6, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# ---------------------------------------------------------------------------
# Main loop: orbit the camera offset pose, pump the sensor each step, and print
# the camera buffer data at each step.
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged render-cadence constant

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        t = sys.GetChTime()                                          # current sim time
        # Orbit the camera around the mesh by recomputing its offset pose.
        cam_x = orbit_radius * math.cos(orbit_rate * t)              # orbit x offset
        cam_y = orbit_radius * math.sin(orbit_rate * t)             # orbit y offset
        new_pose = chrono.ChFramed(
            chrono.ChVector3d(cam_x, cam_y, orbit_height),
            chrono.QuatFromAngleAxis(orbit_rate * t + chrono.CH_PI, chrono.ChVector3d(0, 0, 1)),
        )
        cam.SetOffsetPose(new_pose)                                  # dynamically update the camera pose

        manager.Update()                                            # pump the sensor (once per physics step)
        sys.DoStepDynamics(time_step)                              # advance the dynamics

        # Print the most recent camera buffer data at this step.
        rgba_buffer = cam.GetMostRecentRGBA8Buffer()               # latest RGBA8 frame
        if rgba_buffer.HasData():                                   # only after the first sensor tick
            rgba_data = rgba_buffer.GetRGBA8Data()                 # numpy view of the buffer
            print('Camera buffer received. Resolution: {0}x{1}'.format(
                rgba_buffer.Width, rgba_buffer.Height))
            print('   first pixel RGBA: {0}'.format(rgba_data[0][0]))

        if sys.GetChTime() >= sim_end:
            break
