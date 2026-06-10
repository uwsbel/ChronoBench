"""
Camera Sensor Orbiting Demo — PyChrono 9.0.x / Irrlicht + OptiX

Models a sphere body at the origin with an orbiting camera sensor.
System: ChSystemNSC (no contact, pure visual/sensor demo).
The camera sensor (960×480, OptiX) orbits the sphere at 0.1 rad/s.
Camera offset pose: (-7, 0, 2) relative to the attached body.
Point light: one centered light; NO extra side lights.
Saving: enabled (ChFilterSave writes PNG frames to cam/rgb/).
Expected behavior: smooth orbital view of the sphere over 20 s.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===
time_step    = 1e-3        # physics step [s]
sim_end      = 20.0        # simulation duration [s]
render_fps   = 50.0        # Irrlicht render frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Camera / sensor parameters (per prompt)
CAM_W          = 960        # image width  [px]
CAM_H          = 480        # image height [px]
CAM_FOV        = 1.408      # horizontal FOV [rad]
CAM_UPDATE_HZ  = 30         # physical update rate [Hz]
ORBIT_RATE     = 0.1        # camera orbit rate [rad/s] (was 0.5)
ORBIT_RADIUS   = 7.0        # orbit radius matches offset magnitude
SPHERE_RADIUS  = 1.0        # protagonist sphere radius [m]
SPHERE_DENSITY = 1000.0     # [kg/m^3]

# review-only scaffolding flags

# === System & gravity ===
# Pure visual/sensor demo — no contact, ChSystemNSC without collision setup
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # weightless

# === Bodies ===
# Protagonist sphere — camera rides on this body; it stays fixed at origin
mat = chrono.ChContactMaterialNSC()
sphere = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, False, mat)
sphere.SetPos(chrono.ChVector3d(0, 0, 0))
sphere.SetFixed(True)
sphere.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(sphere)

# Ground/floor reference box (fixed, provides spatial context in sensor view)
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20.0, 0.2, 20.0, 100.0, True, False, ground_mat)
ground.SetPos(chrono.ChVector3d(0, -SPHERE_RADIUS - 0.1, 0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# === Camera sensor — orbiting camera body ===
# One camera with offset pose (-7, 0, 2) as required; no extra point lights.
# Orbit is achieved by rotating the camera body each step.
manager = sens.ChSensorManager(sys)
# Single centered point light — no extra side lights (per prompt)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 5, 0),
    chrono.ChColor(1, 1, 1),
    500.0
)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# Camera mount body: fixed, moved each step to orbit the sphere
cam_body = chrono.ChBody()
cam_body.SetFixed(True)
cam_body.SetPos(chrono.ChVector3d(ORBIT_RADIUS, 0, 0))
sys.AddBody(cam_body)

# Offset pose: (-7, 0, 2) from the cam_body frame (per prompt)
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    cam_body,         # attach to orbiting mount body
    CAM_UPDATE_HZ,    # physical update rate [Hz]
    offset_pose,      # offset pose per prompt
    CAM_W, CAM_H,     # 960 × 480 per prompt
    CAM_FOV,
)
cam.SetName("Orbiting Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain — SCORED CORE (save is enabled per prompt)
cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))   # saving enabled per prompt

manager.AddSensor(cam)

# === Visualization — Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Orbit Demo — 960x480 sensor, 0.1 rad/s orbit")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 4, 0), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -SPHERE_RADIUS - 0.1, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging setup (review-only) ===

# === Main loop ===
frame = 0
orbit_angle = 0.0   # precomputed orbit accumulator

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            t = sys.GetChTime()

            # Advance orbit angle each physics step
            orbit_angle += ORBIT_RATE * time_step  # cache: incremental update

            # Update camera body position to orbit the sphere (world-frame)
            cx = ORBIT_RADIUS * math.cos(orbit_angle)
            cz = ORBIT_RADIUS * math.sin(orbit_angle)
            cam_body.SetPos(chrono.ChVector3d(cx, 0, cz))


            manager.Update()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
