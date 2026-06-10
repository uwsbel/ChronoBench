import os
import math
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------------
# Physical system: NSC system with a fixed box that the camera senses.
# ---------------------------------------------------------------------------
sys = chrono.ChSystemNSC()                                            # rigid-body system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # bullet collision detection

# Box geometry parameters (replaces the prior mesh object).
side = 4.0                                                            # box edge length (m)
box_density = 1000                                                    # density of the box body (kg/m^3)

# Box body the camera is attached to (ChBodyEasyBox: side x side x side).
box_body = chrono.ChBodyEasyBox(side, side, side, box_density)        # cube of given side
box_body.SetPos(chrono.ChVector3d(0, 0, 0))                           # box at the world origin
box_body.SetFixed(True)                                               # box is a static target
box_body.GetVisualShape(0).SetTexture(                                # apply a texture to the box
    chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box_body)                                                    # add the box to the system

# ---------------------------------------------------------------------------
# Sensor manager + scene lighting (point + area lights for the camera render).
# ---------------------------------------------------------------------------
manager = sens.ChSensorManager(sys)                                  # oversees all sensors
intensity = 1.0                                                      # light intensity scalar
manager.scene.AddPointLight(                                         # point light #1
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0)
manager.scene.AddPointLight(                                         # point light #2
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0)
manager.scene.AddAreaLight(                                          # one overhead area light
    chrono.ChVector3f(0, 0, 4),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
    chrono.ChVector3f(1, 0, 0),
    chrono.ChVector3f(0, -1, 0))

# Update the sensor manager's ray-tracing recursions for nicer reflections.
manager.SetRayRecursions(4)                                         # secondary ray bounces

# ---------------------------------------------------------------------------
# Camera sensor parameters (the orbiting camera attached to the box).
# ---------------------------------------------------------------------------
update_rate = 30                                                    # camera update rate (Hz)
image_width = 1280                                                  # image horizontal resolution
image_height = 720                                                  # image vertical resolution
fov = 1.408                                                         # horizontal field of view (rad)
lag = 0                                                             # sensor lag (s)
exposure_time = 0                                                   # collection / exposure window (s)
noise_mean = 0.0                                                    # camera noise mean
noise_stdev = 0.02                                                  # camera noise standard deviation

# Camera offset pose on the box (modified offset: -7, 0, 3), looking back at the box.
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 3),                                    # offset from box in its frame
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))      # slight downward tilt about +Y

# Build the camera sensor attached to the box body.
cam = sens.ChCameraSensor(
    box_body,                                                      # body the camera rides on
    update_rate,                                                   # update rate in Hz
    offset_pose,                                                   # offset pose on the box
    image_width,                                                   # image width
    image_height,                                                  # image height
    fov)                                                           # horizontal FOV
cam.SetName("Camera Sensor")                                       # name the sensor
cam.SetLag(lag)                                                    # set lag
cam.SetCollectionWindow(exposure_time)                             # set exposure window

# Filter chain (order matters; each Save snapshots the buffer at its position).
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(noise_mean, noise_stdev))  # constant-normal noise
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera"))  # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                         # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                      # save stream #1: color PNGs
cam.PushFilter(sens.ChFilterGrayscale())                           # convert to grayscale
cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale Camera"))  # preview of grayscale
cam.PushFilter(sens.ChFilterSave("cam/gray/"))                     # save stream #2: grayscale PNGs
cam.PushFilter(sens.ChFilterImageResize(640, 360))                # resize for downstream consumers
cam.PushFilter(sens.ChFilterR8Access())                           # host access to R8 buffer
manager.AddSensor(cam)                                            # register the camera (filters first)

# ---------------------------------------------------------------------------
# A second, fixed third-person camera on a static helper body for a wide shot.
# ---------------------------------------------------------------------------
tp_body = chrono.ChBody()                                          # fixed helper body for the wide cam
tp_body.SetFixed(True)                                             # static in the world frame
tp_body.SetPos(chrono.ChVector3d(-10, 0, 4))                       # behind and above the box
sys.Add(tp_body)                                                  # add the helper to the system

tp_pose = chrono.ChFramed(                                         # zero offset on the helper body
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0.1, chrono.ChVector3d(0, 1, 0)))     # slight downward tilt
cam_tp = sens.ChCameraSensor(
    tp_body,                                                      # body the wide camera rides on
    update_rate,                                                   # update rate in Hz
    tp_pose,                                                       # offset pose
    image_width,                                                   # image width
    image_height,                                                  # image height
    fov)                                                           # horizontal FOV
cam_tp.SetName("Third Person Camera")                              # name the sensor
cam_tp.SetLag(lag)                                                 # set lag
cam_tp.SetCollectionWindow(exposure_time)                          # set exposure window
cam_tp.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Third Person"))  # wide preview
cam_tp.PushFilter(sens.ChFilterRGBA8Access())                     # host access to RGBA8 buffer
cam_tp.PushFilter(sens.ChFilterSave("cam/third/"))               # save stream #3: third-person PNGs
manager.AddSensor(cam_tp)                                         # register the wide camera

# ---------------------------------------------------------------------------
# Irrlicht review window (separate from the sensor render path).
# ---------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()                          # interactive review window
vis.AttachSystem(sys)                                             # attach the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                # Z-up convention
vis.SetWindowSize(1280, 720)                                     # window size (px)
vis.SetWindowTitle("Camera Sensor on Box")                       # window title
vis.Initialize()                                                # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                  # sky box
vis.AddCamera(chrono.ChVector3d(-12, -6, 6),                     # eye position
              chrono.ChVector3d(0, 0, 0))                        # look at the box
vis.AddTypicalLights()                                           # standard lighting

# ---------------------------------------------------------------------------
# Simulation loop parameters.
# ---------------------------------------------------------------------------
time_step = 1e-3                                                 # integration step (s)
sim_end = 8.0                                                    # total simulated time (s)
render_fps = 30.0                                                # review render cadence (fps)
render_every = max(1, round(1.0 / (render_fps * time_step)))    # physics steps per rendered frame
orbit_radius = 8.0                                              # radius the camera orbits the box
orbit_rate = 0.5                                                # angular rate of the orbit (rad/s)


# Drive a slow orbit of the camera around the box by rotating its offset pose.
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                            # begin the review frame
    vis.Render()                                               # render the Irrlicht scene
    vis.EndScene()                                             # end the review frame
    for _ in range(render_every):
        t = sys.GetChTime()                                    # current sim time
        ang = orbit_rate * t                                   # orbit angle at this time
        cam_pos = chrono.ChVector3d(                           # orbiting camera position
            -orbit_radius * math.cos(ang),
            -orbit_radius * math.sin(ang),
            3.0)
        cam.SetOffsetPose(chrono.ChFramed(                     # aim camera back at the box
            cam_pos,
            chrono.QuatFromAngleAxis(ang, chrono.ChVector3d(0, 0, 1))))
        manager.Update()                                       # pump the sensors once per step
        sys.DoStepDynamics(time_step)                          # advance the dynamics
        buffer = cam.GetMostRecentRGBA8Buffer()                # read the latest camera buffer
        if buffer.HasData():                                   # only after the first sensor tick
            print('Buffer received. Resolution: {0}x{1}'.format(buffer.Width, buffer.Height))
            print('Camera position: ', cam_pos.x, cam_pos.y, cam_pos.z)
        if sys.GetChTime() >= sim_end:
            break
