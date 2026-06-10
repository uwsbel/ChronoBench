import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import os
import math
import time

# Camera parameters
noise_model = "CONST_NORMAL"   # constant normal noise model
update_rate = 30                # camera update rate in Hz (physical rate)
image_width = 1280              # image width in pixels
image_height = 720              # image height in pixels
fov = 1.408                     # horizontal field of view in radians
lag = 0                         # lag between sensing and data accessibility
exposure_time = 0               # camera exposure/collection window in seconds

# Simulation parameters
step_size = 1e-3                # physics step size in seconds
end_time = 20.0                 # simulation end time in seconds
render_fps = 30                 # Irrlicht render rate
render_every = max(1, round(1.0 / (render_fps * step_size)))  # untagged cadence constant

save = True                     # save camera images to disk
vis_sensor = True               # show live sensor preview windows

out_dir = "SENSOR_OUTPUT/"      # output directory for sensor images

# Create the Chrono physical system (non-smooth contact)
mphysicalSystem = chrono.ChSystemNSC()

# Load a triangular mesh from a Wavefront .obj file
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)  # load the HMMWV chassis mesh
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))  # scale mesh uniformly by 2

# Create a visual representation of the mesh
trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)                 # assign the mesh to the shape
trimesh_shape.SetName("HMMWV Chassis Mesh") # name the shape
trimesh_shape.SetMutable(False)              # set to immutable for performance

# Create a fixed body and attach the mesh visual shape to it
mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))   # place at origin
mesh_body.AddVisualShape(trimesh_shape)          # attach visual shape
mesh_body.SetFixed(True)                         # fix the body in space (static mesh)
mphysicalSystem.Add(mesh_body)                   # add body to the physical system

# Create sensor manager to manage all sensors in the simulation
manager = sens.ChSensorManager(mphysicalSystem)

# Add point lights for camera scene illumination
intensity = 1.0                                  # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0,
                            chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))  # area light overhead

# Define the initial camera offset pose relative to the mesh body
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),                                     # initial camera position offset
    chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))          # tilt camera toward mesh
)

# Initialize the camera sensor attached to the mesh body
cam = sens.ChCameraSensor(
    mesh_body,     # body the camera rides on
    update_rate,   # update rate in Hz (physical rate, not 1/dt)
    offset_pose,   # offset pose on the body
    image_width,   # image width in pixels
    image_height,  # image height in pixels
    fov            # horizontal field of view in radians
)
cam.SetName("Camera Sensor")
cam.SetLag(lag)                      # lag between sensing and data accessibility
cam.SetCollectionWindow(exposure_time)  # camera exposure/collection window

# Build the filter chain (ORDER MATTERS — each Save/Access snapshots buffer at its position)
if noise_model == "CONST_NORMAL":
    cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))   # constant normal noise (mean=0, std=0.02)
elif noise_model == "PIXEL_DEPENDENT":
    cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))       # pixel-dependent noise

if vis_sensor:
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))  # live RGB preview

cam.PushFilter(sens.ChFilterRGBA8Access())                           # host access to RGBA8 buffer (scored output)

if save:
    cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))              # save color PNGs (scored core)

cam.PushFilter(sens.ChFilterGrayscale())                             # convert image to grayscale

if vis_sensor:
    cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))  # grayscale preview

if save:
    cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))             # save grayscale PNGs (scored core)

cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))  # resize for downstream
cam.PushFilter(sens.ChFilterR8Access())                              # host access to R8 buffer

manager.AddSensor(cam)                                               # push ALL filters BEFORE AddSensor

# Irrlicht visualization window (review renderer — separate from sensor OptiX renderer)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mphysicalSystem)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor Demo")
vis.Initialize()                                                     # MUST be called FIRST
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-15, 0, 5), chrono.ChVector3d(0, 0, 0))  # Irrlicht window camera
vis.AddTypicalLights()

# Orbit parameters for dynamic camera update
orbit_radius = 10    # radius of camera orbit around the mesh (m)
orbit_rate = 0.5     # angular rate of camera orbit (rad/s)

ch_time = 0.0        # simulation time tracker
t1 = time.time()     # wall-clock start for timing report


while vis.Run() and ch_time < end_time:
    vis.BeginScene(); vis.Render(); vis.EndScene()
    for _ in range(render_every):
        # Dynamically update camera orbit position around the mesh
        cam_x = -orbit_radius * math.cos(ch_time * orbit_rate)       # orbit X
        cam_y = -orbit_radius * math.sin(ch_time * orbit_rate)       # orbit Y
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(cam_x, cam_y, 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))  # orbit pose

        # Access the RGBA8 buffer from the camera and print diagnostic info (scored core)
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))


        manager.Update()                                              # pump sensors every physics step
        mphysicalSystem.DoStepDynamics(step_size)                    # advance simulation by one step
        ch_time = mphysicalSystem.GetChTime()                        # update simulation time
        if ch_time >= end_time:
            break

print("Sim time:", end_time, "Wall time:", time.time() - t1)         # timing report
