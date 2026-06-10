import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

system = chrono.ChSystemNSC()                                        # NSC system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for wheel/ground contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-up gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # contact margin

ground_mat = chrono.ChContactMaterialNSC()                           # rigid-terrain contact material
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 m box terrain
ground.SetPos(chrono.ChVector3d(0, 0, -1))                           # top surface at z=-0.5
ground.SetFixed(True)                                                # static terrain
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # terrain texture
system.Add(ground)                                                   # add ground (rover adds itself)

rover = robot.Viper(system)                                          # built-in Viper rover (owns its bodies)
driver = robot.ViperDCMotorControl()                                 # DC-motor steering driver
rover.SetDriver(driver)                                              # MUST precede Initialize
init_pos = chrono.ChVector3d(0, 0.2, 0)                              # spawn just above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))                # place rover via a ChFramed

manager = sens.ChSensorManager(system)                               # sensor manager owns the camera
intensity = 1.0                                                      # point-light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)  # scene light for the camera

offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1.45),
                              chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))  # POV mount on chassis
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),                                    # mount on the rover chassis
    15,                                                              # update rate (Hz)
    offset_pose,                                                     # camera offset frame
    720,                                                             # image width
    480,                                                             # image height
    1.408,                                                           # horizontal FOV (rad)
)
cam.SetName("Third Person POV")                                      # sensor name
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))  # live POV preview window
cam.PushFilter(sens.ChFilterSave("cam/pov/"))                        # save POV PNG stream (scored output)
manager.AddSensor(cam)                                               # register the camera

vis = chronoirr.ChVisualSystemIrrlicht()                             # real-time Irrlicht window
vis.AttachSystem(system)                                             # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up camera convention
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("Viper rover - Rigid terrain")                    # window title
vis.Initialize()                                                     # Irrlicht init FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()                                               # default lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # pos, aim, radius, near, far, angle, res

time_step = 1e-3                                                     # integration step
max_steering = math.pi / 6                                           # practical steering limit
step_number = 0                                                      # physics step counter for render cadence
render_step_size = 1.0 / 25                                          # FPS = 25
render_steps = math.ceil(render_step_size / time_step)              # steps between rendered frames
time = 0.0                                                           # sim clock for the steering schedule
while vis.Run():                                                     # plain real-time loop
    steering = 0.0                                                   # default: drive straight
    if 2 < time < 7:                                                 # ramp steering in
        steering = max_steering * (time - 2) / 5
    elif 7 < time < 12:                                              # ramp steering back out
        steering = max_steering * (12 - time) / 5
    driver.SetSteering(steering)                                     # command the steering angle
    rover.Update()                                                   # propagate command into the motors
    manager.Update()                                                 # pump the camera sensor
    if (step_number % render_steps == 0):                            # render at the cadence interval
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    step_number += 1                                                 # advance the render counter
    system.DoStepDynamics(time_step)                                 # advance one physics step
    time += time_step                                                # advance the clock
