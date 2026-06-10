"""Camera-sensor demonstration around a box object.

Models a single rigid box (ChBodyEasyBox, 1000 kg/m^3) sensed by an OptiX
ChCameraSensor. The system is ChSystemNSC (rigid, NSC contact). The box is the
only protagonist body; it is fixed in the world while the camera orbits around it
by updating its offset pose each step, producing a 360-degree fly-around RGB +
grayscale image stream. Expected behavior: the box stays put and the recorded
camera video sweeps smoothly around it.
"""

import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Parameters === geometry / camera / timing constants (no bare literals below)
side = 1.0                       # box edge length (m)
box_density = 1000.0             # box density (kg/m^3)
time_step = 1e-3                 # integration step (s)
sim_end = 20.0                   # simulation end time (s)
render_fps = 50.0                # Irrlicht review render cadence (Hz)
update_rate = 30                 # camera physical update rate (Hz) — not 1/dt
image_width = 1280               # camera image width (px)
image_height = 720               # camera image height (px)
fov = 1.408                      # camera horizontal field of view (rad)
orbit_radius = 10.0              # camera orbit radius around the box (m)
orbit_rate = 0.5                 # camera orbit angular rate (rad/s)
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === System & collision === rigid NSC world; Bullet collision for the box shape
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === the box object the camera senses (fixed, textured)
box = chrono.ChBodyEasyBox(side, side, side, box_density)
box.SetPos(chrono.ChVector3d(0, 0, 0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(box)

# === Sensor manager & lighting === point + area lights for the camera render
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0,
                           chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

# === Camera sensor === attached to the box, offset behind & above, RGB + gray streams
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 3),
    chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(box, update_rate, offset_pose, image_width, image_height, fov)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
cam.PushFilter(sens.ChFilterGrayscale())
cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale"))
cam.PushFilter(sens.ChFilterSave("cam/gray/"))
cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))
cam.PushFilter(sens.ChFilterR8Access())
manager.AddSensor(cam)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera sensor orbiting a box")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-9, -9, 6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === orbit the camera each step; render + capture review video

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            cam.SetOffsetPose(chrono.ChFramed(
                chrono.ChVector3d(-orbit_radius * math.cos(t * orbit_rate),
                                  -orbit_radius * math.sin(t * orbit_rate), 1),
                chrono.QuatFromAngleAxis(t * orbit_rate, chrono.ChVector3d(0, 0, 1))))
            manager.Update()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback; traceback.print_exc()
    raise
