import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC()


gravity = chrono.ChVectorD(0, -9.81, 0)
chrono.ChSystemNSC.Set_G_acc(gravity)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
chrono.ChSystemNSC.Add(ground)


viper_rover = Viper()  
viper_rover.SetPos(chrono.ChVectorD(0, 0.5, 0))  
chrono.ChSystemNSC.Add(viper_rover)


driver = ViperDriver(viper_rover)  
chrono.ChSystemNSC.Add(driver)


application = chronoirr.ChIrrApp(chrono.ChSystemNSC, "Viper Rover Simulation", chronoirr.dimension2d(800, 600))
application.AddSkyBox()
application.AddCamera(chrono.ChVector3(0, 2, -5), chrono.ChVector3(0, 0, 0))
application.AddLight(chrono.ChVector3(0, 10, -10), chrono.ChVector3(1, 1, 1))


time_step = 0.01  
total_time = 10.0  
steering_duration = 5.0  
initial_steering = 0.0  
final_steering = 30.0  


current_time = 0.0
while current_time < total_time:
    
    if current_time <= steering_duration:
        steering_angle = initial_steering + (final_steering - initial_steering) * (current_time / steering_duration)
        driver.SetSteeringAngle(chrono.ChDegreeToRad(steering_angle))  

    
    chrono.ChSystemNSC.DoStepDynamics(time_step)

    
    application.DrawAll()
    application.Run()

    
    current_time += time_step


application.Close()