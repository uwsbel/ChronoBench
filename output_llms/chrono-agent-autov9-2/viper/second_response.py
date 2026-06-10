"""Viper rover driving on flat rigid ground, filmed by an onboard third-person camera.

Models the NASA/JPL-style Viper four-wheel rover (pychrono.robot.Viper) on a
ChSystemNSC world with a single flat rigid-ground patch the wheels make contact
with. The rover is driven by a ViperDCMotorControl driver (DC drive motors plus
steering); a small constant steering angle makes the rover follow a gentle arc so
its motion is clearly visible. A chassis-mounted camera sensor renders a
third-person POV that follows the rover.

System type: NSC (non-smooth contact) — wheels vs. ground use rigid contact.
Main bodies: the Viper rover (chassis + 4 wheels + suspension arms/uprights) and
a fixed ground box.
Expected behavior: the rover accelerates from rest and drives forward along a
curved path; the chassis position advances over the simulated duration.
"""

import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / timing (no bare positional literals downstream)
time_step = 1e-3                      # s, NSC integration step
sim_end = 12.0                        # s, total simulated duration
render_fps = 25.0                     # frames per second for the review window
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once: physics steps per frame

ground_size = 100.0                   # m, side length of the flat ground patch
ground_thickness = 1.0                # m, ground box thickness
ground_top_z = 0.0                    # m, top surface of the ground (rover drives on this)

ground_friction = 0.8                 # rover-vs-ground friction coefficient
ground_restitution = 0.0              # inelastic contact

rover_init_z = ground_top_z + 0.2     # m, chassis spawn height above ground so wheels rest on it
rover_init_pos = chrono.ChVector3d(0.0, 0.0, rover_init_z)
rover_init_rot = chrono.QuatFromAngleZ(0.0)   # facing +X at spawn

steering_angle = 0.18                 # rad, gentle constant steering -> visible curved path

# camera (third-person POV) parameters, fixed in the chassis local frame
cam_update_rate = 15.0                # Hz
cam_width = 720                       # px
cam_height = 480                      # px
cam_fov = 1.408                       # rad, horizontal field of view
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0.0, 1.45),                       # behind/above the chassis origin
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),  # slight downward tilt
)

# === System & gravity === NSC world with Bullet narrow-phase collision for wheel/ground contact
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required: wheels contact the ground

# === Ground === single fixed rigid patch; the rover wheels make NSC contact with this
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(ground_friction)
ground_mat.SetRestitution(ground_restitution)

ground = chrono.ChBodyEasyBox(
    ground_size, ground_size, ground_thickness,
    1000.0,        # density (irrelevant: body is fixed)
    True,          # visualization
    True,          # collision
    ground_mat,
)
ground.SetPos(chrono.ChVector3d(0, 0, ground_top_z - 0.5 * ground_thickness))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Rover === Viper four-wheel rover driven by a DC-motor controller with constant steering
driver = robot.ViperDCMotorControl()         # DC drive motors + steering controller
rover = robot.Viper(system)                   # rover registers its bodies/joints on `system`
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(rover_init_pos, rover_init_rot))
rover.Update()                                # settle internal state after Initialize

chassis_part = rover.GetChassis()             # cache: chassis part fetched once, reused for camera + logging
chassis_body = chassis_part.GetBody()         # cache: underlying ChBody of the chassis

# apply a steady steering command so the rover follows a visible arc
driver.SetSteering(steering_angle)

# === Sensor === chassis-mounted third-person POV camera (OptiX); Irrlicht is the review window
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

cam = sens.ChCameraSensor(
    chassis_body,        # rides on the rover chassis -> follows the rover (third-person POV)
    cam_update_rate,
    cam_offset_pose,
    cam_width,
    cam_height,
    cam_fov,
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "Viper Front Camera"))
cam.PushFilter(sens.ChFilterSave("cam/sensor_pov/"))   # PNG frames -> per-sensor mp4
cam.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(cam)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover — Third-Person POV Camera")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-3.0, -3.0, 2.5), chrono.ChVector3d(0, 0, 0.3))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === advance physics + sensors; render the review window at the frame cadence

frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                          # pump the camera sensor every physics step
            rover.Update()                            # advance the rover's driver/controller state
            system.DoStepDynamics(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:             # solver divergence / invalid simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
