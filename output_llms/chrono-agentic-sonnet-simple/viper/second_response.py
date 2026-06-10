import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

system = chrono.ChSystemNSC()  # NSC for rover contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # bullet collision required
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)  # collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)  # collision margin

ground_mat = chrono.ChContactMaterialNSC()  # NSC contact for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # flat ground plane
ground.SetPos(chrono.ChVector3d(0, 0, -1))  # top surface at z=0
ground.SetFixed(True)  # fixed ground
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
system.Add(ground)

rover = robot.Viper(system)  # Viper 6-wheel rover
driver = robot.ViperDCMotorControl()  # DC motor driver for steering
rover.SetDriver(driver)  # attach driver before Initialize
init_pos = chrono.ChVector3d(0, 0.2, 0)  # rover spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity rotation
rover.Initialize(chrono.ChFramed(init_pos, init_rot))  # Initialize with frame

# sensor manager and camera sensor
manager = sens.ChSensorManager(system)  # sensor manager
intensity = 1.0  # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)  # scene light

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1.45),
    chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))  # camera offset on chassis
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),  # attach to rover chassis
    15,  # update rate Hz
    offset_pose,
    720,  # image width
    480,  # image height
    1.408  # field of view (rad)
)
cam.SetName("Third Person POV")  # sensor name
cam.SetLag(0)  # zero lag
cam.SetCollectionWindow(0)  # zero exposure
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())  # host buffer access
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))  # save sensor frames (scored core)
manager.AddSensor(cam)  # add camera to manager

vis = chronoirr.ChVisualSystemIrrlicht()  # Irrlicht window
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Z-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover - Camera Sensor")
vis.Initialize()  # MUST be called before adding scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # overhead view
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)  # shadow light

time_step = 1e-3  # simulation step
sim_end = 20.0  # simulation end time
render_fps = 25.0  # render frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))  # render cadence

step_number = 0  # step counter
render_step_size = 1.0 / 25  # FPS = 25
render_steps = math.ceil(render_step_size / time_step)  # steps between renders

max_steering = math.pi / 6  # max steering angle


while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()  # current sim time

    # ramp steering
    steering = 0.0
    if 2 < time < 7:
        steering = max_steering * (time - 2) / 5  # ramp up
    elif 7 < time < 12:
        steering = max_steering * (12 - time) / 5  # ramp down
    driver.SetSteering(steering)  # apply steering

    rover.Update()  # propagate command to motors

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    manager.Update()  # update sensor manager every step
    system.DoStepDynamics(time_step)  # advance physics


    step_number += 1
