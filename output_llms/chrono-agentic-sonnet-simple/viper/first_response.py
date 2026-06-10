import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                        # NSC for rover contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for wheel-terrain contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81)) # g = 9.81 down (Z-down)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # collision margin

ground_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # flat terrain box
ground.SetPos(chrono.ChVector3d(0, 0, -1))                           # top surface at z=0
ground.SetFixed(True)                                                # fixed ground
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

rover = robot.Viper(system)                                          # Viper rover (owns bodies)
driver = robot.ViperDCMotorControl()                                 # DC-motor steering driver
rover.SetDriver(driver)                                              # attach driver BEFORE Initialize
init_pos = chrono.ChVector3d(0, 0.2, 0)                             # spawn above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity rotation
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # Initialize takes ChFramed

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up convention
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - Rigid terrain")
vis.Initialize()                                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # shadow light

time_step = 1e-3                                                     # physics time step
sim_end = 20.0                                                       # total simulation time
steering_ramp_time = 10.0                                            # time period over which steering changes
max_steering = math.pi / 6                                           # max steering angle ~0.52 rad
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # render cadence (untagged)


while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()
        # gradually ramp steering from 0 to max_steering over steering_ramp_time
        if time < steering_ramp_time:
            steering = max_steering * (time / steering_ramp_time)   # ramp up steering angle
        else:
            steering = max_steering                                  # hold at maximum steering
        driver.SetSteering(steering)                                 # update steering on driver
        rover.Update()                                               # propagate steering to motors
        system.DoStepDynamics(time_step)
        if system.GetChTime() >= sim_end:
            break
