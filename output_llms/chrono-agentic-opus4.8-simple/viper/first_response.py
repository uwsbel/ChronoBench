import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

time_step = 1e-3                                                      # integration step
sim_end = 14.0                                                        # total sim time

system = chrono.ChSystemNSC()                                         # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact -> Bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g, Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # small envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small margin

ground_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # rigid terrain box
ground.SetPos(chrono.ChVector3d(0, 0, -1))                           # top surface at z = -0.5
ground.SetFixed(True)                                                # static terrain
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # texture
system.Add(ground)                                                   # add ground to system

rover = robot.Viper(system)                                          # built-in Viper rover
driver = robot.ViperDCMotorControl()                                 # DC-motor steering driver
rover.SetDriver(driver)                                              # attach driver BEFORE Initialize
init_pos = chrono.ChVector3d(0, 0, 0)                                # spawn at origin
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # Initialize takes a ChFramed

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht window
vis.AttachSystem(system)                                             # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                   # Z-up camera
vis.SetWindowSize(1280, 720)                                         # window size
vis.SetWindowTitle("Viper rover - Rigid terrain")                  # title
vis.Initialize()                                                     # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()                                               # standard lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # shadow-casting light

max_steering = math.pi / 6                                           # practical steering max (rad)
render_fps = 50.0                                                    # review-video cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                                 # begin frame
    vis.Render()                                                     # draw scene
    vis.EndScene()                                                   # end frame
    for _ in range(render_every):
        t = system.GetChTime()                                       # current sim time
        steering = 0.0                                               # straight by default
        if 2 < t < 7:
            steering = max_steering * (t - 2) / 5                     # gradually steer in
        elif 7 < t < 12:
            steering = max_steering * (12 - t) / 5                    # gradually steer back
        driver.SetSteering(steering)                                 # command steering
        rover.Update()                                               # propagate command to motors
        system.DoStepDynamics(time_step)                             # advance physics
        if system.GetChTime() >= sim_end:
            break
