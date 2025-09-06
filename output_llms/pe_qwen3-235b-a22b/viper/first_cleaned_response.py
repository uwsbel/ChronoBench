import pychrono as chrono
import pychrono.robot as viper
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, -1, 0))
ground.SetFixed(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(20, 1, 20)  
ground.SetCollide(True)


ground_shape = chrono.ChVisualShapeBox(20, 1, 20)
ground_shape.SetColor(chrono.ChColor(0, 1, 0))  
ground.AddVisualShape(ground_shape)
system.Add(ground)


rover = viper.Viper(system)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))  


driver = viper.ViperDriver()
rover.SetDriver(driver)


driver.SetThrottle(0.5)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover on Rigid Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -3))  
vis.AddTypicalLights()


time_step = 0.01
end_time = 10.0
steering_amplitude = 0.5  


while vis.Run():
    current_time = system.GetChTime()
    
    
    if current_time < end_time:
        steering = steering_amplitude * (current_time / end_time)
        driver.SetSteering(steering)
    
    
    rover.Update()
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)