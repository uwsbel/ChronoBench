import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Added sensor import

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)
# Set ground texture (assuming the visual shape exists)
if ground.GetVisualShape(0):
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Viper rover
rover = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation (fixed ChFrameD)
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected ChFrameD

# Create sensor manager and add point light
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0
)

# Create camera sensor (fixed ChFrameD and quaternion)
offset_pose = chrono.ChFrameD(
    chrono.ChVector3d(1.0, 0, 1.45),
    chrono.Q_from_AngAxis(0.2, chrono.ChVector3d(0, 1, 0))  # Corrected quaternion function
)
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),
    offset_pose,
    720,
    480,
    1.408,
    0.1,  # Near plane
    100.0  # Far plane
)
cam.SetName("Third Person POV")
cam.SetUpdateRate(15)  # Set update rate separately
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
manager.AddSensor(cam)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)

time_step = 1e-3
step_number = 0
render_step_size = 1.0 / 25  # 25 FPS
render_steps = math.ceil(render_step_size / time_step)

# Simulation loop
time = 0
while vis.Run():
    time += time_step
    steering = 0
    max_steering = math.pi / 6
    
    if 2 < time < 7:
        steering = max_steering * (time - 2) / 5
    elif 7 < time < 12:
        steering = max_steering * (12 - time) / 5
    
    driver.SetSteering(steering)
    rover.Update()
    
    manager.Update()  # Update sensor manager
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    step_number += 1
    
    system.DoStepDynamics(time_step)