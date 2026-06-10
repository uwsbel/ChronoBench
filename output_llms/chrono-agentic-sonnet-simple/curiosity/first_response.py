import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact scene needs bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81)) # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # collision margin

# rigid ground — fixed box, top surface at z=0 for Curiosity spawn
ground_mat = chrono.ChContactMaterialNSC()                           # NSC material for rovers
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                        # top at z=0 (Curiosity sits lower)
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Curiosity rover with DC-motor control driver
rover = robot.Curiosity(system)                                       # rover owns its bodies
driver = robot.CuriosityDCMotorControl()                             # DC motor steering driver
rover.SetDriver(driver)                                              # must be before Initialize
init_pos = chrono.ChVector3d(0, 0.2, 0)                             # spawn above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity quaternion
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # takes ChFramed, not bare pos

# Irrlicht visualization — Initialize first, scene elements after
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up world convention
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover - Rigid Terrain")
vis.Initialize()                                                     # FIRST, then add scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo after initialize
vis.AddSkyBox()                                                      # sky box after initialize
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()                                               # standard two-light setup
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),          # shadow light position
                       chrono.ChVector3d(0, 0, 0.5),               # aim point
                       3, 4, 10, 40, 512)                           # radius, near, far, angle, res
# simulation parameters
time_step = 1e-3                                                     # 1 ms physics step
sim_end = 20.0                                                       # 20 s total run
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))        # frames between renders
max_steering = math.pi / 6                                          # max ±30 deg steering


while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()
        # ramp steering: straight → turn in → back out for real-time steering demo
        steering = 0.0
        if 2 < time < 7:
            steering = max_steering * (time - 2) / 5               # ramp up steering
        elif 7 < time < 12:
            steering = max_steering * (12 - time) / 5              # ramp down steering
        driver.SetSteering(steering)                                # set steering each step
        rover.Update()                                              # propagate command to motors
        system.DoStepDynamics(time_step)                           # advance physics
        if system.GetChTime() >= sim_end:
            break
