import os
import math
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC system for the sensed scene
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # Bullet collision system

# --- mesh to be sensed by the camera (scaled HMMWV chassis) ---
mmesh = chrono.ChTriangleMeshConnected()                              # triangle mesh container
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)  # load OBJ
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))    # scale up by 2

trimesh_shape = chrono.ChVisualShapeTriangleMesh()                    # visual shape for the mesh
trimesh_shape.SetMesh(mmesh)                                          # bind the loaded mesh
trimesh_shape.SetName("HMMWV Chassis Mesh")                           # shape name
trimesh_shape.SetMutable(False)                                       # static geometry

mesh_body = chrono.ChBody()                                           # body carrying the mesh
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                          # placed at the origin
mesh_body.AddVisualShape(trimesh_shape)                               # attach the mesh visual
mesh_body.SetFixed(True)                                              # fixed in the world
sys.Add(mesh_body)                                                    # add to the system

# --- sensor manager + scene lighting (camera-only point/area lights) ---
manager = sens.ChSensorManager(sys)                                   # oversees all sensors
intensity = 1.0                                                       # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)  # single point light
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))  # area light

# --- camera sensor parameters ---
update_rate = 30                                                      # physical update rate (Hz)
image_width = 960                                                     # image width (px)
image_height = 480                                                    # image height (px)
fov = 1.408                                                           # horizontal field of view (rad)
lag = 0                                                               # lag (s)
exposure_time = 0                                                     # exposure / collection window (s)
save = True                                                           # save camera images to PNG

# --- build the camera sensor on the mesh body ---
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),                                      # offset from the body
    chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))          # tilt about Y
cam = sens.ChCameraSensor(
    mesh_body,                                                        # body the camera is attached to
    update_rate,                                                      # update rate in Hz
    offset_pose,                                                      # offset pose
    image_width,                                                      # image width
    image_height,                                                     # image height
    fov)                                                              # horizontal field of view
cam.SetName("Camera Sensor")                                          # sensor name
cam.SetLag(lag)                                                       # sensing lag
cam.SetCollectionWindow(exposure_time)                               # exposure window

# --- filter chain (ORDER MATTERS; each Save snapshots the buffer at its position) ---
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))  # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                           # host access to RGBA8 buffer
if save:
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                    # save RGB PNGs
cam.PushFilter(sens.ChFilterGrayscale())                            # convert to grayscale
cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))  # grayscale preview
if save:
    cam.PushFilter(sens.ChFilterSave("cam/gray/"))                   # save grayscale PNGs
cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))  # resize
cam.PushFilter(sens.ChFilterR8Access())                            # host access to R8 buffer
manager.AddSensor(cam)                                               # push all filters before adding

# --- Irrlicht window (review render path; separate from the sensor) ---
vis = chronoirr.ChVisualSystemIrrlicht()                             # interactive window
vis.AttachSystem(sys)                                                # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up
vis.SetWindowSize(1280, 720)                                         # window size
vis.SetWindowTitle("Camera Sensor")                                 # window title
vis.Initialize()                                                     # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))  # window camera
vis.AddTypicalLights()                                               # standard lights

# --- orbit + simulation parameters ---
orbit_radius = 10                                                    # orbit radius (m)
orbit_rate = 0.1                                                     # orbit angular rate (rad/s)
step_size = 1e-3                                                     # physics step (s)
end_time = 20.0                                                      # simulation end time (s)
render_fps = 30.0                                                    # review render cadence (fps)
render_every = max(1, round(1.0 / (render_fps * step_size)))        # steps per rendered frame


while vis.Run() and sys.GetChTime() < end_time:
    vis.BeginScene()                                                # begin frame
    vis.Render()                                                    # render scene
    vis.EndScene()                                                  # end frame
    for _ in range(render_every):
        ch_time = sys.GetChTime()                                  # current sim time
        cam.SetOffsetPose(chrono.ChFramed(                          # orbit the camera around the mesh
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                              -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()              # most recent RGBA8 frame
        if rgba8_buffer.HasData():                                  # only read once the sensor has ticked
            rgba8_data = rgba8_buffer.GetRGBA8Data()              # access pixel data
            print('RGBA8 buffer recieved from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))
        manager.Update()                                           # pump sensors once per step
        sys.DoStepDynamics(step_size)                              # advance dynamics
        if sys.GetChTime() >= end_time:
            break
