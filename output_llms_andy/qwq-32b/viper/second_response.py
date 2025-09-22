import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.sensor as sens  # Added sensor import
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystemType.BULLET)  # Corrected enum access
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Set collision parameters on the system's collision system
system.GetCollisionSystem().SetDefaultSuggestedEnvelope(0.0025)
system.GetCollisionSystem().SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChMaterialSurfaceNSC()  # Using material surface instead of ContactMaterialNSC
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Viper rover
rover = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected ChFrameD instead of ChFramed

# Create run-time visualization
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
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Sensor manager setup
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100), chrono.ChColor(  # Corrected ChVectorF instead of ChVector3f
    intensity, intensity, intensity), 500.0)

# Camera sensor setup
offset_pose = chrono.ChFrameD(
    chrono.ChVectorD(1.0, 0, 1.45),
    chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0))  # Corrected quaternion creation
)
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),
    15,  # update rate
    offset_pose,
    720,  # image width
    480,  # image height
    1.408  # field of view
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
manager.AddSensor(cam)

# Simulation step variables
time_step = 1e-3
step_number = 0
render_step_size = 1.0 / 25  # FPS = 25
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
    
    # Render at intervals
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    step_number += 1
    
    system.DoStepDynamics(time_step)