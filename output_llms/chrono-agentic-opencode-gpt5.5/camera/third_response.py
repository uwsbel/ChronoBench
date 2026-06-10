"""Camera sensor demo with an NSC rigid scene, a textured box, and an RGB camera attached to the box.

The simulation creates a fixed floor and a box body with collision geometry so both Irrlicht and the
OptiX camera can render it. The camera rides on the slowly moving box at the requested offset and
records an RGB stream while the attachment target remains visible.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === named parameters keep the camera, body, and timing easy to verify
time_step = 1.0e-3
sim_end = 5.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

side = 1.0
box_density = 1000.0
floor_size = 12.0
floor_thickness = 0.1
box_pos = chrono.ChVector3d(0.0, 0.0, side / 2.0)
camera_offset = chrono.ChVector3d(-7.0, 0.0, 3.0)
camera_update_rate = 30.0
image_width = 1280
image_height = 720
camera_fov = 1.408


# === System & gravity === NSC contact handles the box and floor collision geometry
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.8)
contact_mat.SetRestitution(0.0)


# === Bodies === a box replaces the mesh object and gives the sensor collision-backed geometry
floor = chrono.ChBodyEasyBox(floor_size, floor_size, floor_thickness, box_density, True, True, contact_mat)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, 0.0, -floor_thickness / 2.0))
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box = chrono.ChBodyEasyBox(side, side, side, box_density, True, True, contact_mat)
box.SetFixed(False)
box.SetPos(box_pos)
box.SetLinVel(chrono.ChVector3d(0.25, 0.0, 0.0))
box.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 0.4))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

box_body = box  # cache: camera attach body and logger reuse this handle each step


# === Sensor camera === the prompt-required camera is attached to the box at the final offset pose
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 8.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-4.0, -3.0, 6.0),
    chrono.ChColor(0.8, 0.8, 0.8),
    500.0,
)

offset_pose = chrono.ChFramed(
    camera_offset,
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
cam = sens.ChCameraSensor(box_body, camera_update_rate, offset_pose, image_width, image_height, camera_fov)
cam.SetName("Box RGB Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)


# === Visualization === Irrlicht window is separate from the OptiX camera sensor
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Box-Attached Camera Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, -7.0, 3.5), chrono.ChVector3d(0.0, 0.0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(
    1.0,
    1.0,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main loop === render once per frame, update the camera and physics in fixed time steps

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
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid numeric state
    traceback.print_exc()
    raise
finally:
    pass
