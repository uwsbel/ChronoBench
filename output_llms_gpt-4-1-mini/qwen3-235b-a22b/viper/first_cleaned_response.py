import pychrono as chrono
import pychrono.viper as viper
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(100, 1, 100)  
ground.SetPos(chrono.ChVectorD(0, -1, 0))  
system.Add(ground)


rover = viper.ChViper(system)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))  


driver = viper.ChViperDriver()
rover.SetDriver(driver)


application = irrlicht.ChIrrApp(system, 'Viper Rover Simulation', irrlicht.dimension2du(1280, 720))
application.AddTypicalLogo()
application.AddTypicalCamera(irrlicht.vector3df(0, 3, -6), irrlicht.vector3df(0, 0, 0))  
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


end_time = 10.0  
max_steering = 1.0  


while application.GetDevice().run():
    current_time = system.GetChTime()
    
    
    if current_time <= end_time:
        steering = -max_steering + (2 * max_steering) * (current_time / end_time)
    else:
        steering = max_steering  
    
    
    driver.SetSteering(steering)
    driver.SetThrottle(0.3)  

    
    system.DoStepDynamics(1e-3)  

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()