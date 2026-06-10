"""
Camera sensor demo — PyChrono 9.0.x + Irrlicht + OptiX sensor camera.

Models a spinning box body in an NSC system with a chase camera that orbits
around the protagonist at 0.1 rad/s. The ChCameraSensor (960x480, 30 Hz)
saves RGB frames to cam/rgb/ and displays a live preview. Scene uses only
a single ambient point light (no extra illumination lights). Expected
behavior: the box body rotates and the sensor camera orbits it, continuously
saving images.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Simulation constants ===
TIME_STEP = 1e-3       # physics step [s]
SIM_END   = 10.0       # simulation duration [s]
RENDER_FPS = 50.0      # Irrlicht render rate [fps]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Camera sensor parameters
CAM_W      = 960       # image width [px]
CAM_H      = 480       # image height [px]
CAM_FOV    = 1.408     # horizontal FOV [rad]
CAM_RATE   = 30        # physical update rate [Hz]
ORBIT_RATE = 0.1       # camera orbit rate [rad/s]

# Box body parameters
BOX_X = 1.0
BOX_Y = 1.0
BOX_Z = 1.0
BOX_DENSITY = 1000.0   # [kg/m³]

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Bodies ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)
mat.SetRestitution(0.0)

# Ground floor (fixed)
ground = chrono.ChBodyEasyBox(10.0, 10.0, 0.2, 1000.0, True, True, mat)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))
sys.AddBody(ground)

# Protagonist box that the sensor camera rides on
box_body = chrono.ChBodyEasyBox(BOX_X, BOX_Y, BOX_Z, BOX_DENSITY, True, True, mat)
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
box_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.5))
sys.AddBody(box_body)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
# Single ambient point light well above the scene — no extra illumination lights
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(1, 1, 1),
    500.0
)

# Camera offset pose: -7 m behind, 2 m above, slight pitch toward subject
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)

# OptiX camera sensor (960x480, 30 Hz) attached to the protagonist box body
cam = sens.ChCameraSensor(
    box_body,       # attach to the REAL protagonist body
    CAM_RATE,       # physical update rate [Hz] — NOT 1/time_step
    offset_pose,    # offset frame in body coordinates
    CAM_W, CAM_H,   # width, height [px]
    CAM_FOV,        # horizontal FOV [rad]
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain: visualize → access → save (saving enabled)
cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))

manager.AddSensor(cam)

# === Visualization — full Irrlicht block (Initialize FIRST, scene elements AFTER) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -6, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()  # cache: fetched once per inner step

            # Orbit the camera around the box body at ORBIT_RATE rad/s
            angle = ORBIT_RATE * sim_time  # precomputed orbit angle
            orbit_x = -7.0 * math.cos(angle)
            orbit_y = -7.0 * math.sin(angle)
            new_offset = chrono.ChFramed(
                chrono.ChVector3d(orbit_x, orbit_y, 2.0),
                chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
            )
            cam.SetOffsetPose(new_offset)

            manager.Update()

            # Guard sensor buffer access (empty before first sensor tick)
            buf = cam.GetMostRecentRGBA8Buffer()
            if buf.HasData():  # guard: skip frames the sensor hasn't filled yet
                pass           # buffer available; downstream processing would go here

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
