"""PyChrono camera sensor scene using an NSC rigid-body system.

The script builds a small fixed-object scene, attaches an RGB camera sensor to a
rotating camera rig, and saves camera images from a 960x480 stream. The camera
orbits the scene at 0.1 rad/s from an offset of (-7, 0, 2), while Irrlicht
renders the same scene for interactive inspection.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants ===
TIME_STEP = 0.001
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

CAMERA_UPDATE_RATE = 30.0
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 480
CAMERA_FOV = 1.408
CAMERA_ORBIT_RATE = 0.1
SAVE_CAMERA_IMAGES = True

GROUND_SIZE_X = 12.0
GROUND_SIZE_Y = 8.0
GROUND_THICKNESS = 0.2
BOX_DENSITY = 1000.0


# === System & Materials ===
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.GetSolver().AsIterative().SetMaxIterations(80)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.8)
contact_mat.SetRestitution(0.05)


# === Bodies ===
ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X,
    GROUND_SIZE_Y,
    GROUND_THICKNESS,
    BOX_DENSITY,
    True,
    True,
    contact_mat,
)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, -GROUND_THICKNESS / 2.0))
system.Add(ground)

scene_boxes = []
for idx, (x_pos, y_pos, height) in enumerate(
    [(-2.0, -1.0, 0.8), (0.0, 1.1, 1.2), (2.2, -0.4, 1.0)]
):
    box = chrono.ChBodyEasyBox(0.7, 0.7, height, BOX_DENSITY, True, True, contact_mat)
    box.SetFixed(True)
    box.SetPos(chrono.ChVector3d(x_pos, y_pos, height / 2.0))
    box.SetName(f"static_camera_subject_{idx}")
    system.Add(box)
    scene_boxes.append(box)

camera_rig = chrono.ChBody()
camera_rig.SetName("orbiting_camera_rig")
camera_rig.SetFixed(True)
camera_rig.SetPos(chrono.ChVector3d(0, 0, 0.8))
system.AddBody(camera_rig)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor Orbit")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(7, -7, 4), chrono.ChVector3d(0, 0, 0.8))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    24,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Sensor Camera ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)

camera = sens.ChCameraSensor(
    camera_rig,
    CAMERA_UPDATE_RATE,
    offset_pose,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
)
camera.SetName("Orbit RGB Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "RGB Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
if SAVE_CAMERA_IMAGES:
    camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(camera)

rig_body = camera_rig  # cache: fetched once, reused every step
rgb_camera = camera  # cache: sensor handle reused for guarded buffer checks


# === Main Loop ===
def main():
    frame = 0

    try:

            while vis.Run() and system.GetChTime() < SIM_END:
                sim_time = system.GetChTime()
                orbit_angle = CAMERA_ORBIT_RATE * sim_time
                rig_body.SetRot(chrono.QuatFromAngleAxis(orbit_angle, chrono.ChVector3d(0, 0, 1)))

                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                frame += 1

                for _ in range(RENDER_EVERY):
                    sim_time = system.GetChTime()
                    orbit_angle = CAMERA_ORBIT_RATE * sim_time
                    rig_body.SetRot(chrono.QuatFromAngleAxis(orbit_angle, chrono.ChVector3d(0, 0, 1)))
                    manager.Update()
                    rgba_buffer = rgb_camera.GetMostRecentRGBA8Buffer()
                    camera_has_data = 1 if rgba_buffer.HasData() else 0
                    system.DoStepDynamics(TIME_STEP)
                    if system.GetChTime() >= SIM_END:
                        break

    except (OSError, IOError) as exc:  # disk or permission failure while writing review data
        print(f"review data output failed: {exc}")
        raise
    except (RuntimeError, ValueError) as exc:  # simulation or sensor update failure
        print(f"simulation failed: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
