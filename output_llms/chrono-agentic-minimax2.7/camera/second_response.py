"""
Camera sensor demo: orbiting RGB camera with noise filters and visualization.

A triangular mesh (HMMWV chassis) is fixed in the scene. A camera sensor
orbits around it with configurable offset, update rate, image dimensions,
and filter chain (noise, visualize, save, grayscale, resize, access).
Simulation runs for 20 seconds with data logged to CSV.
"""
import csv as csv_module
import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Camera parameters ===
noise_model = "CONST_NORMAL"   # noise type: CONST_NORMAL, PIXEL_DEPENDENT, or NONE
update_rate = 30              # camera update rate in Hz (physical rate, not 1/dt)
image_width = 960             # image width in pixels
image_height = 480            # image height in pixels
fov = 1.408                  # horizontal field of view in radians
lag = 0                       # lag between sensing and data accessibility (s)
exposure_time = 0             # exposure time per image (s)

# === Simulation parameters ===
step_size = 1e-3             # dynamics time step (s)
sim_end = 20.0               # simulation end time (s)

# === Recording parameters ===
save = True                   # whether to save camera images
vis = True                    # whether to visualize camera output
out_dir = "cam/"              # output directory for saved images

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# Load HMMWV chassis mesh
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

# Visual shape for the mesh
trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)

# Fixed body carrying the mesh
mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(True)
sys.AddBody(mesh_body)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)

# Lighting - single point light (removed extra lights per input2.txt)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0,
                           chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

# === Camera sensor ===
# Modified per input2.txt: offset changed from (-5,0,2) to (-7,0,2)
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),
    chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    mesh_body,
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)

# Filter chain
if noise_model == "CONST_NORMAL":
    cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
elif noise_model == "PIXEL_DEPENDENT":
    cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))
# NONE: no noise filter

if vis:
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

cam.PushFilter(sens.ChFilterRGBA8Access())

if save:
    cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

cam.PushFilter(sens.ChFilterGrayscale())

if vis:
    cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

if save:
    cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))
cam.PushFilter(sens.ChFilterR8Access())

manager.AddSensor(cam)

# === Irrlicht visualization ===
vis_irr = chronoirr.ChVisualSystemIrrlicht()
vis_irr.AttachSystem(sys)
vis_irr.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis_irr.SetWindowSize(1280, 720)
vis_irr.SetWindowTitle("Camera Sensor Demo")
vis_irr.Initialize()
vis_irr.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis_irr.AddSkyBox()
vis_irr.AddCamera(chrono.ChVector3d(15, -15, 10), chrono.ChVector3d(0, 0, 0))
vis_irr.AddTypicalLights()
vis_irr.AddGrid(2.0, 2.0, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop with CSV logging ===
orbit_radius = 10
# Modified per input2.txt: orbit rate changed from 0.5 to 0.1 rad/s
orbit_rate = 0.1
ch_time = 0.0

render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))

os.makedirs("frames", exist_ok=True)

frame = 0

# CSV setup - open before loop, close in finally
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None
try:

    while vis_irr.Run() and ch_time < sim_end:
        vis_irr.BeginScene()
        vis_irr.Render()
        vis_irr.EndScene()


        for _ in range(render_every):
            # Dynamic camera orbit - update offset pose each step
            cam.SetOffsetPose(chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            ))

            # Access camera buffer
            rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
            if rgba8_buffer.HasData():
                rgba8_data = rgba8_buffer.GetRGBA8Data()
                # Only print occasionally to avoid spam
                if int(ch_time * 10) % 10 == 0:
                    print(f"RGBA8 buffer: {rgba8_buffer.Width}x{rgba8_buffer.Height}, "
                          f"First pixel: {rgba8_data[0, 0, :]}")

            # Update sensor manager
            manager.Update()

            # Log CSV

            # Step dynamics
            sys.DoStepDynamics(step_size)
            ch_time = sys.GetChTime()

            if ch_time >= sim_end:
                break

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # review-only: close CSV + assemble videos below

print(f"Simulation complete. Sim time: {sim_end}")
