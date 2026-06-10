import math                                                            # trig for the orbit
import pychrono.core as chrono                                        # core PyChrono
import pychrono.sensor as sens                                        # sensor module (OptiX)
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer

# ---------------------------------------------------------------------------
# Physical system
# ---------------------------------------------------------------------------
sys = chrono.ChSystemNSC()                                           # non-smooth contact system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # bullet collision detection

# ---------------------------------------------------------------------------
# Mesh body the camera observes (loaded from the bundled vehicle HMMWV body)
# ---------------------------------------------------------------------------
mmesh = chrono.ChTriangleMeshConnected()                             # triangle mesh container
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)  # load OBJ
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2.0))  # scale the mesh up 2x

trimesh_shape = chrono.ChVisualShapeTriangleMesh()                  # visual mesh shape
trimesh_shape.SetMesh(mmesh)                                         # bind the loaded mesh
trimesh_shape.SetName("HMMWV Chassis Mesh")                         # shape name
trimesh_shape.SetMutable(False)                                     # static mesh (no deformation)

mesh_body = chrono.ChBody()                                          # the body the mesh rides on
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                         # placed at the world origin
mesh_body.AddVisualShape(trimesh_shape)                             # attach the visual mesh
mesh_body.SetFixed(True)                                            # fixed scene object
sys.Add(mesh_body)                                                  # add to the system

# ---------------------------------------------------------------------------
# Sensor manager + scene lighting (camera-only point/area lights)
# ---------------------------------------------------------------------------
manager = sens.ChSensorManager(sys)                                 # oversees all sensors
manager.scene.AddPointLight(                                        # single base point light
    chrono.ChVector3f(2, 2.5, 100),                                # light position (ChVector3f)
    chrono.ChColor(1, 1, 1),                                        # white light
    500.0,                                                         # range
)
manager.scene.AddAreaLight(                                         # an area light for soft fill
    chrono.ChVector3f(0, 0, 4),                                     # light position
    chrono.ChColor(1, 1, 1),                                        # white
    500.0,                                                         # range
    chrono.ChVector3f(1, 0, 0),                                     # in-plane x axis
    chrono.ChVector3f(0, -1, 0),                                    # in-plane y axis
)

# ---------------------------------------------------------------------------
# Camera sensor parameters
# ---------------------------------------------------------------------------
update_rate = 30                                                    # camera update rate (Hz)
image_width = 960                                                   # image width  (px)
image_height = 480                                                  # image height (px)
fov = 1.408                                                         # horizontal field of view (rad)
exposure_time = 0.0                                                 # collection window (s)
orbit_radius = 7.0                                                  # camera orbit radius (m)
orbit_rate = 0.1                                                    # camera orbit rate (rad/s)

# camera offset pose relative to the mesh body
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),                                    # offset from the body frame
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),        # no initial tilt
)

cam = sens.ChCameraSensor(
    mesh_body,                                                     # body the camera rides on
    update_rate,                                                  # physical update rate (Hz)
    offset_pose,                                                  # offset frame on the body
    image_width,                                                  # image width
    image_height,                                                 # image height
    fov,                                                          # horizontal FOV (rad)
)
cam.SetName("Camera Sensor")                                       # sensor name
cam.SetLag(0)                                                      # no lag
cam.SetCollectionWindow(exposure_time)                            # exposure/collection window

# filter chain (ORDER MATTERS) — noise, preview, host access, and PNG save
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))     # constant-normal sensor noise
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera"))  # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                        # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                     # SAVE the RGB frames (save=True)
manager.AddSensor(cam)                                            # register the camera (after filters)

# ---------------------------------------------------------------------------
# Irrlicht visualization window
# ---------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht visual system
vis.AttachSystem(sys)                                             # attach the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                # Z-up vertical
vis.SetWindowSize(1280, 720)                                     # window size
vis.SetWindowTitle("Camera Sensor Demo")                        # window title
vis.Initialize()                                                # initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                  # sky box
vis.AddCamera(chrono.ChVector3d(-8, 0, 3), chrono.ChVector3d(0, 0, 0))  # window camera
vis.AddTypicalLights()                                          # standard lights

# ---------------------------------------------------------------------------
# Time-stepping + orbit
# ---------------------------------------------------------------------------
time_step = 1e-3                                                  # integration step (s)
sim_end = 40.0                                                    # end time (s)
render_fps = 30.0                                                # render cadence (Hz)
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged cadence constant

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                            # begin Irrlicht frame
    vis.Render()                                               # render the window
    vis.EndScene()                                            # end Irrlicht frame
    for _ in range(render_every):
        t = sys.GetChTime()                                   # current sim time
        # orbit the camera around the mesh at orbit_rate rad/s
        cam_x = orbit_radius * math.cos(orbit_rate * t)        # orbiting x offset
        cam_y = orbit_radius * math.sin(orbit_rate * t)        # orbiting y offset
        cam.SetOffsetPose(                                     # move the camera each step
            chrono.ChFramed(
                chrono.ChVector3d(cam_x, cam_y, 2),            # orbiting position (z = 2)
                chrono.QuatFromAngleAxis(orbit_rate * t + chrono.CH_PI, chrono.ChVector3d(0, 0, 1)),  # aim inward
            )
        )
        manager.Update()                                      # pump the sensor once per step
        sys.DoStepDynamics(time_step)                         # advance the physics
        # read the most recent RGBA8 buffer (truth-style sensor data output)
        buffer = cam.GetMostRecentRGBA8Buffer()               # latest camera buffer
        if buffer.HasData():                                  # skip ticks the sensor hasn't filled
            print('Buffer received. Resolution: {0}x{1}'.format(buffer.Width, buffer.Height))
        if sys.GetChTime() >= sim_end:
            break
