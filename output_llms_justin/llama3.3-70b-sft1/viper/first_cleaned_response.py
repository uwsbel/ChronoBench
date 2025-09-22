import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.robot as robot


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)

system.Add(ground)


driver = robot.ViperDCMotorControl()
rover = robot.Viper(system)
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(chrono.ChVector3d(0, -0.2, 0), chrono.ChQuaterniond(1, 0, 0, 0)))


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)


steering_time = 1.0  
max_steering_angle = 0.6
time = 0
time_step = 1e-3
time_end = 30

while time < time_end:
    time = system.GetChTime()

    
    if time < steering_time:
        driver.SetSteering(0, max_steering_angle * time / steering_time)
    else:
        driver.SetSteering(0, max_steering_angle)

    driver.SetSpeed(0, 0.8)

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)