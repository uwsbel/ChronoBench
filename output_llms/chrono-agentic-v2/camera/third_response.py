"""
Camera sensor demo — box protagonist with attached OptiX camera.

System: ChSystemNSC (Y-up gravity).
Bodies: a dynamic box (ChBodyEasyBox) acting as the main protagonist.
Sensors: a ChCameraSensor attached to the box at offset (-7, 0, 3),
         capturing RGB frames via ChFilterSave + Irrlicht window for review.
Expected behavior: box falls under gravity; the camera rides with it,
         continuously capturing the scene from behind and above.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===
time_step    = 1e-3        # physics time step [s]
sim_end      = 10.0        # simulation end time [s]
render_fps   = 50.0        # Irrlicht render cadence [Hz]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Box geometry / physics
side        = 1.0          # box side length [m]
box_density = 1000.0       # box density [kg/m³]
box_init_y  = 2.0          # initial Y height [m]

# Camera sensor parameters
cam_update_rate = 30       # physical update rate [Hz]
cam_width, cam_height = 1280, 720
cam_fov = 1.408            # horizontal FOV [rad]


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.6)
mat.SetRestitution(0.2)

# === Bodies ===
# Box protagonist — dynamic body that falls under gravity
box_body = chrono.ChBodyEasyBox(side, side, side, box_density, True, True, mat)
box_body.SetPos(chrono.ChVector3d(0, box_init_y, 0))
box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box_body)

# Fixed ground plane so the box has something to land on
ground = chrono.ChBodyEasyBox(10.0, 0.1, 10.0, 1000.0, True, True, mat)
ground.SetPos(chrono.ChVector3d(0, -0.05, 0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# === Sensor manager + camera sensor ===
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 3),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    box_body,
    cam_update_rate,
    offset_pose,
    cam_width, cam_height,
    cam_fov,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Demo — Box Protagonist")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-10, 5, 0), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# === Main loop ===
os.makedirs("frames", exist_ok=True)  # guard against missing output dir

frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame += 1
        for _ in range(render_every):
            manager.Update()
            t   = sys.GetChTime()   # cache: avoid repeated getter calls
            pos = box_body.GetPos()  # cache: fetched once per step
            vel = box_body.GetPosDt()  # cache: fetched once per step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad input state
    import traceback; traceback.print_exc()
    raise
finally:
    pass  # CSV writer closed in review-only block below
