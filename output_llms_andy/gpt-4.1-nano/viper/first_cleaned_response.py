import pychrono.core as chrono
import pychrono.vehicle as veh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
system.Add(ground)




viper = veh.Viper(system)


viper.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
viper.SetChassisPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
viper.SetChassisFixed(False)


driver = veh.ChIrrGuiDriver()
driver.Initialize()
viper.GetVehicle().SetDriver(driver)


application = veh.ChIrrApp(viper.GetVehicle(), 'Viper Rover Simulation', chrono.dimension(800, 600))
application.GetSceneManager().AddCamera(chrono.ChCameraSceneNode(
    application.GetSceneManager(), 
    application.GetVideoDriver(), 
    chrono.ChVectorD(0, 2, -4),  
    chrono.ChVectorD(0, 0, 0),   
    30                          
))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.01
total_time = 10.0
steering_rate = 0.1  
max_steering_angle = 0.5  
current_steering = 0.0
steering_direction = 1  


t = 0
while t < total_time:
    
    current_steering += steering_direction * steering_rate * time_step
    if current_steering > max_steering_angle:
        current_steering = max_steering_angle
        steering_direction = -1
    elif current_steering < -max_steering_angle:
        current_steering = -max_steering_angle
        steering_direction = 1
    
    
    viper.GetVehicle().SetSteeringAngle(current_steering)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    system.DoStepDynamics(time_step)
    application.GetVideoDriver().run()
    application.GetVideoDriver().waitFrame()

    t += time_step