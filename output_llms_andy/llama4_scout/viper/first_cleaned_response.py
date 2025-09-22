import chrono as ch
import chrono.irrlicht as chirr
import math


ch.ChEngine.Initialize()


system = ch.ChSystemNSC()
system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


ground = ch.ChBodyEasyTerrain(
    100, 100,  20, 20, 
    ch.ChVector3d(0, -0.1, 0), 
    ch.ChQuaterniond(1, 0, 0, 0),
    False, False
)
ground.SetPos(ch.ChVector3d(0, -0.2, 0))
system.AddBody(ground)


viper = ch.ChVehicle()
viper.Initialize(ch.ChVector3d(0, 0.5, 0), system)


driver = ch.ChVehicleDriver(viper)
driver.SetSteeringDelta(0.0)  


app = chirr.ChIrrApp(system, "Viper Rover Simulation")
app.SetCameraPosition(ch.ChVector3d(0, 5, -10))
app.SetCameraTarget(ch.ChVector3d(0, 0, 0))


app.EnableDefaultLights()
app.SetVerbose(False)


timestep = 0.05
end_time = 10.0  
steering_change_time = 5.0  
initial_steering = 0.0
final_steering = 30.0  


t = 0
while t < end_time:
    
    steering = initial_steering + (final_steering - initial_steering) * (t / steering_change_time) if t < steering_change_time else final_steering
    driver.SetSteeringDelta(math.radians(steering))
    
    
    system.Update(timestep)
    
    
    app.Render()
    app.SpinEventLoop(0.02)
    
    
    t += timestep


del app
ch.ChEngine.Finalize()