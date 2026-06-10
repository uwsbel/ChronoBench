"""Curiosity Mars rover navigating on rigid terrain (PyChrono 9.0.0, Irrlicht).

Models the built-in rocker-bogie `robot.Curiosity` rover driven by a DC-motor
steering controller (`robot.CuriosityDCMotorControl`) on a fixed, collidable
rigid ground box. The system is non-smooth (`ChSystemNSC`) with Bullet collision,
Z-up gravity. The rover rolls forward under its always-on DC drive while the
steering command is ramped to exercise real-time steering inputs; the Irrlicht
window provides the customizable camera, lighting, shadows, textures, and logo.

Expected behavior: the rover spawns on top of the ground, drives forward, and
steers smoothly through the ramped command without falling through the terrain.
"""

import math

import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / control parameters
time_step = 1e-3                  # rover integration step (Viper/Curiosity)
sim_end = 14.0                    # total simulated time (s)
render_fps = 50.0                 # render cadence (frames per simulated second)
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
max_steering = math.pi / 6       # practical steering limit (rad)
ground_z = -0.5                  # Curiosity ground: 1 m box -> top at z=0
init_pos = chrono.ChVector3d(0, 0, -0.24)      # rover spawn: wheels rest on terrain top
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    # identity (w,x,y,z)

# === System & gravity === NSC + Bullet collision (wheel<->terrain contact)
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground === fixed, collidable rigid terrain box with a concrete texture
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, ground_z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Rover === built-in Curiosity, DC-motor steering driver, system-owned bodies
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)                          # SetDriver BEFORE Initialize
rover.Initialize(chrono.ChFramed(init_pos, init_rot))
chassis_body = rover.GetChassis().GetBody()      # cache: chassis fetched once for logging

# === Visualization === full Irrlicht scene: window + sky + camera + lights + shadow
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - Rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3.0, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)   # pos, aim, radius, near, far, angle, resolution

# === Main loop === drive forward, ramp steering, step physics in real time

try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = system.GetChTime()

            # ramp steering: straight, turn in, then back out
            steering = 0.0
            if 2 < t < 7:
                steering = max_steering * (t - 2) / 5      # ramp up
            elif 7 < t < 12:
                steering = max_steering * (12 - t) / 5     # ramp down
            driver.SetSteering(steering)
            rover.Update()        # propagate steering into the rover's motors

            system.DoStepDynamics(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries plot, then clean frames
