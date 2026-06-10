import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

time_step = 1e-3                                              # integration step (s)
sim_end = 14.0                                                # total simulated time (s)

system = chrono.ChSystemNSC()                                # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # contact needs Bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)  # small rover contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)    # small rover contact margin

ground_mat = chrono.ChContactMaterialNSC()                   # NSC contact material for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)   # 20x20x1 ground box
ground.SetPos(chrono.ChVector3d(0, 0, -1))                   # top face at z=-0.5
ground.SetFixed(True)                                        # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))   # texture
system.Add(ground)                                           # add ground to the system

rover = robot.Viper(system)                                  # built-in 6-wheel Viper rover
driver = robot.ViperDCMotorControl()                         # DC-motor steering driver
rover.SetDriver(driver)                                      # attach driver BEFORE Initialize
init_pos = chrono.ChVector3d(0, 0, 0)                        # spawn at origin
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                  # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))        # Initialize takes a ChFramed

manager = sens.ChSensorManager(system)                       # sensor manager on the rover system
intensity = 1.0                                              # point-light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)   # scene light

offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1.45),
                              chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))   # cam offset
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),                            # mount on the rover chassis
    15,                                                      # update rate (Hz)
    offset_pose,                                             # offset pose on the chassis
    720,                                                     # image width
    480,                                                     # image height
    1.408,                                                   # field of view (rad)
)
cam.SetName("Third Person POV")                              # camera name
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))   # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                   # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))         # save RGB frames (scored output)
manager.AddSensor(cam)                                       # register camera with the manager

vis = chronoirr.ChVisualSystemIrrlicht()                     # Irrlicht review window
vis.AttachSystem(system)                                     # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)            # Z-up camera
vis.SetWindowSize(1280, 720)                                 # window size
vis.SetWindowTitle("Viper rover - Third Person POV camera")  # window title
vis.Initialize()                                             # device first, scene after
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                              # sky box
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))   # eye, target
vis.AddTypicalLights()                                       # standard lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                    # pos, aim, radius, near, far, angle, res

max_steering = math.pi / 6                                   # practical max steering (rad)
render_fps = 25.0                                            # review render cadence (fps)
render_every = max(1, round(1.0 / (render_fps * time_step)))   # physics steps per frame (untagged)

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                         # begin frame
    vis.Render()                                             # draw the scene
    vis.EndScene()                                           # end frame
    for _ in range(render_every):
        t = system.GetChTime()                               # current sim time
        steering = 0.0                                        # default: drive straight
        if 2 < t < 7:
            steering = max_steering * (t - 2) / 5            # ramp steering in
        elif 7 < t < 12:
            steering = max_steering * (12 - t) / 5           # ramp steering out
        driver.SetSteering(steering)                         # command the steering
        rover.Update()                                       # propagate command to motors
        manager.Update()                                     # pump the camera sensor
        chassis_pos = rover.GetChassis().GetBody().GetPos()  # chassis position
        system.DoStepDynamics(time_step)                     # advance physics
        if system.GetChTime() >= sim_end:
            break
