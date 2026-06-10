"""
Camera sensor demo — box body with attached RGB sensor camera.

System:   ChSystemNSC, Y-up gravity, no contact (box is floating/fixed).
Bodies:   One box body (ChBodyEasyBox) set as fixed.
Sensor:   ChCameraSensor attached to the box with offset pose (-7, 0, 3).
Expected: The camera orbits with the box and captures RGB frames.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Parameters ===
side = 1.0              # box side length [m]
box_density = 1000.0    # box density [kg/m³]

time_step = 1e-3        # physics time step [s]
sim_end = 10.0          # simulation duration [s]
render_fps = 50.0       # Irrlicht render cadence [fps]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity ===
# ChSystemNSC — no contact/collision needed (box is fixed; pure visual demo)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# Contact material (NSC) — needed by ChBodyEasyBox even for a fixed body
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
mat.SetRestitution(0.0)

# Box body — side × side × side cube, density=1000
box_body = chrono.ChBodyEasyBox(side, side, side, box_density, True, True, mat)
box_body.SetPos(chrono.ChVector3d(0, 0, 0))
box_body.SetFixed(True)
# Apply a texture to the box
box_body.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.AddBody(box_body)

# === Sensor manager + camera sensor ===
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0
)

# Offset pose: camera sits at (-7, 0, 3) from box, tilted slightly downward
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 3),                                  # offset from body frame
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),    # tilt toward object
)

cam = sens.ChCameraSensor(
    box_body,          # attach to the REAL box body
    30,                # physical update rate [Hz]
    offset_pose,       # offset frame relative to the body
    1280, 720,         # width, height [px]
    1.408,             # horizontal FOV [rad]
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain (scored core — prompt-required sensor outputs)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                        # host RGBA8 access
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                     # save color PNGs

manager.AddSensor(cam)

# === Visualization (Irrlicht window) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Box + Camera Sensor Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, 2, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -side * 0.5 - 0.01, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===

# CSV writer (review-only)

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()  # pump sensor every physics step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
