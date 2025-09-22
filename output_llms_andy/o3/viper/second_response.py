import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.sensor as sens                     # 1.  sensor module
from   pychrono import irrlicht as chronoirr


# ------------------------------------------------
# 1. Create Chrono system
# ------------------------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# gravity --------------------------------------------------------------
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))   # correct API name

# default collision margins -------------------------------------------
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# ------------------------------------------------
# 2. Ground body
# ------------------------------------------------
ground_mat = chrono.ChContactMaterialNSC()
ground     = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos  (chrono.ChVector3d(0, 0, -1))       # plate top is at z = 0
ground.SetFixed(True)

# visual appearance ----------------------------------------------------
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(ground)


# ------------------------------------------------
# 3. Viper rover and driver
# ------------------------------------------------
rover  = viper.Viper(system)                       # build rover
driver = viper.ViperDCMotorControl()               # simple DC-motor driver
rover.SetDriver(driver)

# initial pose ---------------------------------------------------------
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize( chrono.ChFrameD(init_pos, init_rot) )


# ------------------------------------------------
# 4. Irrlicht run-time visualisation
# ------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover – rigid terrain')
vis.Initialize()
vis.AddLogo      (chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera    (chrono.ChVector3d(0, 2.5, 1.5),
                  chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)
# vis.EnableShadows()   # uncomment for shadows


# ------------------------------------------------
# 5.  Sensor manager + camera                          (new section)
# ------------------------------------------------
manager   = sens.ChSensorManager(system)

# point light for the sensor scene ------------------
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 1.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0
)

# third-person camera --------------------------------
offset_pose = chrono.ChFrameD(
    chrono.ChVector3d(1.0, 0, 1.45),
    chrono.Q_from_AngAxis(0.2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),   # parent body
    15,                             # update rate [Hz]
    offset_pose,
    720,                            # width  [px]
    480,                            # height [px]
    1.408                           # FOV [rad]
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
manager.AddSensor(cam)


# ------------------------------------------------
# 6.  Time stepping parameters
# ------------------------------------------------
time_step         = 1e-3                    # dynamics step size [s]
time              = 0.0
step_number       = 0
render_step_size  = 1.0 / 25                # render at 25 FPS
render_steps      = math.ceil(render_step_size / time_step)


# ------------------------------------------------
# 7.  Simulation loop
# ------------------------------------------------
while vis.Run():
    # ----- steering profile -------------------------------------------
    steering      = 0.0
    max_steering  = math.pi / 6            # 30°
    if   2 < time < 7:
        steering = max_steering * (time - 2) / 5.0
    elif 7 < time < 12:
        steering = max_steering * (12 - time) / 5.0
    driver.SetSteering(steering)

    # ----- advance subsystems -----------------------------------------
    rover.Update()                         # update rover subsystem
    manager.Update()                       # update all attached sensors

    # ----- render at chosen FPS ---------------------------------------
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # ----- dynamics step ----------------------------------------------
    system.DoStepDynamics(time_step)
    time        += time_step
    step_number += 1