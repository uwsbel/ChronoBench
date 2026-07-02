import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


rover = vehicle.Viper(system)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.5), chrono.ChQuaternionD(1, 0, 0, 0)))
driver = vehicle.ChDriver(rover)
driver.SetThrottle(1.0)  


application = irr.ChIrrApp(system, 'Viper Rover Simulation', irr.dimension2d_uint(800, 600))
application.SetSkyBox()
application.SetCamera(chrono.ChVectorD(0, -10, 3), chrono.ChVectorD(0, 0, 1.5))
application.SetLightDirection(chrono.ChVectorD(0, 0, -1), True)
application.AddTypicalLogo()
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(0.01)
application.SetTryRealtime(True)


steering_magnitude = 0.5  
steering_time = 5.0  


while application.GetDevice().run():
    application.BeginScene()
    
    
    current_time = system.GetChTime()
    if current_time < steering_time:
        steering = (current_time / steering_time) * steering_magnitude
    else:
        steering = steering_magnitude
    driver.SetSteering(steering)
    
    
    application.DoStep()
    
    
    application.DrawAll()
    application.EndScene()