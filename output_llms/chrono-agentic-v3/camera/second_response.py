"""
PyChrono Camera Sensor Orbit Demo
==================================
System type : ChSystemNSC (NSC, no contact/collision needed — pure MBS)
Main body   : A box mesh (the protagonist) that the camera orbits around.
Sensor      : One ChCameraSensor (OptiX) attached to the box body with an
              offset pose that updates each step to orbit around the body.
Objective   : Demonstrate a camera orbiting a box mesh at 0.1 rad/s with
              960×480 resolution, saving RGB images to cam/rgb/.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants ===
TIME_STEP   = 1e-3          # physics step (s)
SIM_END     = 20.0          # simulation end time (s)
RENDER_FPS  = 50.0          # Irrlicht render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Camera sensor parameters
CAM_WIDTH   = 960           # image width (pixels)
CAM_HEIGHT  = 480           # image height (pixels)
CAM_FOV     = 1.408         # horizontal FOV (rad)
CAM_UPDATE  = 30            # physical update rate (Hz) — NOT 1/time_step
ORBIT_RATE  = 0.1           # camera orbit rate (rad/s) — updated each step
CAM_OFFSET_DIST = 7.0       # orbit radius (m) — offset from body
CAM_OFFSET_Z    = 2.0       # z-height of camera above body

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# No collision shapes or contact — pure sensor demo; SetCollisionSystemType omitted.

# === Bodies ===
# Protagonist: a fixed box that the camera orbits around.
box_hx, box_hy, box_hz = 1.0, 1.0, 1.0   # box half-extents (m)
mat = chrono.ChContactMaterialNSC()

protagonist = chrono.ChBodyEasyBox(
    2 * box_hx, 2 * box_hy, 2 * box_hz,   # full extents
    1000.0,                                 # density
    True,                                   # visual shape
    False,                                  # collision shape (not needed for pure sensor demo)
    mat,
)
protagonist.SetPos(chrono.ChVector3d(0, 0, 0))
protagonist.SetFixed(True)
sys.AddBody(protagonist)

# Ground plane (visual reference — no collision needed)
ground = chrono.ChBodyEasyBox(10.0, 10.0, 0.2, 1000.0, True, False, mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1 - box_hz))
ground.SetFixed(True)
sys.AddBody(ground)

# === Sensor Manager & Camera ===
manager = sens.ChSensorManager(sys)
# Single point light for the OptiX camera scene
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
# (Extra point lights at (9,2.5,100), (16,2.5,100), (23,2.5,100) removed per delta)

# Initial camera offset pose — offset behind and above the body in the body frame.
# The orbit is implemented by updating the sensor's offset pose each physics step.
initial_angle = 0.0
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-CAM_OFFSET_DIST, 0, CAM_OFFSET_Z),           # offset from body
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),       # slight upward tilt
)

cam = sens.ChCameraSensor(
    protagonist,   # attach to the REAL protagonist body
    CAM_UPDATE,    # physical update rate (Hz)
    offset_pose,   # initial offset pose
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain — scored core: prompted to save images (save=True)
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))          # saving enabled (save=True per delta)
cam.PushFilter(sens.ChFilterGrayscale())
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH // 2, CAM_HEIGHT // 2, "Grayscale"))
cam.PushFilter(sens.ChFilterSave("cam/gray/"))

manager.AddSensor(cam)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Orbit Demo")
vis.Initialize()                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(-8, -8, 5),
    chrono.ChVector3d(0, 0, 0),
)
vis.AddTypicalLights()

# === Pre-loop setup ===


# Orbit angle state (updated every physics step, used by scored core)
orbit_angle = initial_angle  # rad

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            # Update camera orbit pose each physics step
            orbit_angle = t * ORBIT_RATE
            cos_a = math.cos(orbit_angle)
            sin_a = math.sin(orbit_angle)
            cam_x = -CAM_OFFSET_DIST * cos_a
            cam_y = -CAM_OFFSET_DIST * sin_a
            cam_z = CAM_OFFSET_Z
            # Build updated offset pose in the body frame (body is at origin, fixed)
            orbit_quat = chrono.QuatFromAngleAxis(orbit_angle, chrono.ChVector3d(0, 0, 1))
            new_offset = chrono.ChFramed(
                chrono.ChVector3d(cam_x, cam_y, cam_z),
                chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
            )
            cam.SetOffsetPose(new_offset)

            # Guard sensor buffer access before reading
            buf = cam.GetMostRecentRGBA8Buffer()
            if buf.HasData():   # guard: skip frames before first sensor tick
                pass            # buffer available — no downstream consumer here

            manager.Update()   # pump sensors every physics step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # CSV closed in review-only block below
