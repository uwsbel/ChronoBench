import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC()


chrono.ChSystemNSC.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetMaterialSurface().SetFriction(0.5)
chrono.ChSystemNSC.Add(ground)


viper_rover = chrono.ChBody()
viper_rover.SetMass(1000)  
viper_rover.SetInertiaXX(chrono.ChVectorD(10, 10, 10))  
viper_rover.SetPos(chrono.ChVectorD(0, 0.5, 0))  
viper_rover.GetMaterialSurface().SetFriction(0.5)
chrono.ChSystemNSC.Add(viper_rover)


driver = chrono.ChDriver()
driver.SetChassis(viper_rover)


application = chronoirr.ChIrrApp(chrono.ChSystemNSC, "Viper Rover Simulation", chronoirr.dimension2d(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))


time_step = 0.01  
total_time = 10.0  
current_time = 0.0


initial_steering = 0.0
final_steering = 30.0  
steering_duration = 5.0  
steering_rate = (final_steering - initial_steering) / steering_duration


while current_time < total_time:
    
    if current_time < steering_duration:
        current_steering = initial_steering + steering_rate * current_time
    else:
        current_steering = final_steering

    
    driver.SetSteering(current_steering)

    
    chrono.ChSystemNSC.DoStepDynamics(time_step)

    
    application.DrawAll()
    application.DoStep()

    
    current_time += time_step


application.AssetBindAll()
application.AssetUpdateAll()
application.GetDevice().drop()