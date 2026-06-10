"""Camera-sensor demo: an RGB camera rides on a textured cube.

Models a single rigid textured box (side x side x side, density 1000 kg/m^3)
resting on a fixed ground plate in an NSC system under -Z gravity. A
``sens.ChCameraSensor`` is rigidly attached to the box through an offset frame
of ChVector3d(-7, 0, 3) relative to the box body, so the camera follows the box
and views it from behind-and-above. The box is given Bullet collision geometry
so the OptiX sensor renderer (which only draws bodies with collision shapes) can
see it and so it makes contact with the ground.

System type: NSC (ChSystemNSC). Main body: the textured cube. Expected
behavior: the cube rests in stable static contact on the ground; the attached
camera produces a steady RGB view of the textured cube from its offset pose.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants === geometry / physics / timing (no bare literals downstream)
time_step = 1e-3                       # s, integration step
sim_end = 3.0                          # s, total simulated time
render_fps = 30.0                      # Hz, Irrlicht review-frame cadence
side = 1.0                             # m, cube edge length
box_density = 1000.0                   # kg/m^3, cube material density
ground_size = 20.0                     # m, square ground plate edge
ground_thickness = 1.0                 # m, ground plate thickness
box_z = side / 2.0                     # cube center height so it rests on ground top
cam_offset = chrono.ChVector3d(-7, 0, 3)   # camera position relative to the cube
cam_width, cam_height = 1280, 720      # sensor image resolution
cam_fov = 1.408                        # rad, horizontal field of view
update_rate = 30.0                     # Hz, sensor capture cadence (steady scene)
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity === NSC system with Bullet collision (cube contacts ground)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material === shared NSC material for ground + cube
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies === fixed ground plate + the textured cube (the camera's subject)
ground = chrono.ChBodyEasyBox(ground_size, ground_size, ground_thickness,
                              box_density, True, True, mat)
ground.SetPos(chrono.ChVector3d(0, 0, -ground_thickness / 2.0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

box = chrono.ChBodyEasyBox(side, side, side, box_density, True, True, mat)
box.SetPos(chrono.ChVector3d(0, 0, box_z))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box)
assert box.GetPos().z >= side / 2.0 - 1e-6, "cube must start resting on the ground"

# === Sensor manager === oversees the camera; lighting for the OptiX render path
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-5, 5, 20), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# === Camera sensor === attached to the cube at offset (-7, 0, 3), aimed at the cube
forward = (chrono.ChVector3d(0, 0, 0) - cam_offset).GetNormalized()  # look toward the cube center
look_at_quat = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward)
cam = sens.ChCameraSensor(
    box,                                              # rides on the cube body
    update_rate,                                      # Hz, from 1/time_step
    chrono.ChFramed(cam_offset, look_at_quat),        # offset pose on the cube
    cam_width, cam_height,
    cam_fov,
)
cam.PushFilter(sens.ChFilterVisualize(cam_width, cam_height))   # live preview window
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))           # PNG frames -> sensor mp4
cam.PushFilter(sens.ChFilterRGBA8Access())                     # frame-buffer access
manager.AddSensor(cam)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera on a textured cube")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-7, 4, 3), chrono.ChVector3d(0, 0, box_z))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))


# === Main loop === render once per frame; step physics + pump the sensor per step
box_body = box   # cache: main body handle reused every step
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()            # pump the camera sensor every physics step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
