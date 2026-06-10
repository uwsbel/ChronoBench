"""
Camera Sensor Demo — Box Object.

Models a box body rotating in place, with a ChCameraSensor attached to it.
The sensor camera follows the box, capturing RGB and grayscale streams.
System type: ChSystemNSC (Y-up, gravity disabled for the spinning box demo).
Main body: a box (ChBodyEasyBox) spinning about Y-axis.
Expected behavior: the box rotates continuously; the sensor camera orbits with it,
producing color and grayscale image streams.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===
time_step  = 1e-3          # physics step size [s]
sim_end    = 10.0          # simulation end time [s]
render_fps = 50.0          # Irrlicht render rate [Hz]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

side = 1.0                 # box side length [m]
box_density = 1000.0       # box density [kg/m³]
spin_rate = 1.0            # angular velocity about Y [rad/s]

# Camera offset: behind and above the box in the box's local frame
cam_offset_pos = chrono.ChVector3d(-7, 0, 3)
cam_tilt_angle = 0.2       # tilt about Y axis [rad]
cam_update_rate = 30       # physical update rate [Hz]
cam_width, cam_height = 1280, 720
cam_fov = 1.408            # horizontal FOV [rad]

# Irrlicht window camera position / target
irr_eye    = chrono.ChVector3d(-8, -6, 4)
irr_target = chrono.ChVector3d(0, 0, 0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # gravity off for spinning box

# === Bodies ===
# Ground / reference body (fixed, not visible — structural anchor)
ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

# Box body: replaces the mesh body; camera attaches here
mat = chrono.ChContactMaterialNSC()
box_body = chrono.ChBodyEasyBox(side, side, side, box_density, True, False)
box_body.SetPos(chrono.ChVector3d(0, 0, 0))
box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
box_body.SetAngVelParent(chrono.ChVector3d(0, spin_rate, 0))  # spin about Y
sys.AddBody(box_body)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor Demo — Box")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(irr_eye, irr_target)
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -1.0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Sensor Manager & Camera ===
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0
)

# Attach camera to the box body; offset pose places it behind and above
offset_pose = chrono.ChFramed(
    cam_offset_pos,
    chrono.QuatFromAngleAxis(cam_tilt_angle, chrono.ChVector3d(0, 1, 0)),
)

cam = sens.ChCameraSensor(
    box_body,           # attach to the real protagonist body (the box)
    cam_update_rate,    # physical Hz
    offset_pose,
    cam_width, cam_height,
    cam_fov,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain: color preview → access → save, then grayscale preview → save
cam.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
cam.PushFilter(sens.ChFilterGrayscale())
cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale"))
cam.PushFilter(sens.ChFilterSave("cam/gray/"))
cam.PushFilter(sens.ChFilterImageResize(640, 360))
cam.PushFilter(sens.ChFilterR8Access())
manager.AddSensor(cam)

# === Review-only setup ===

# CSV logging setup

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad parameter
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
