import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


system = ch.ChSystemNSC()
system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


ground = ch.ChBodyEasyTerrain(
    100, 100,  20, 20,  
    ch.GetChronoDataFile("terrain/terrain.jpg"),  
    ch.ChVector3d(0, 0, 0),  
    False  
)
system.AddBody(ground)


viper = ch.ChVehicle(ch.GetChronoDataFile("vehicles/viper/Viper.json"))
system.Add(viper.GetChassisBody())
for wheel in viper.GetWheels():
    system.Add(wheel)
for susp in viper.GetSuspensionLinks():
    system.Add(susp)


driver = ch.ChVehicleDriver(viper)
driver.SetSpeed(5)  


vis = irr.ChIrrApp(system, 'Viper Rover Simulation', 800, 600)
vis.SetCameraPosition(ch.ChVector3d(0, -10, 5))
vis.SetCameraTarget(ch.ChVector3d(0, 0, 0))
vis.EnableCollisionVisualization(True)
vis.EnableSFX(True)
vis.EnableShadows(True)


t_end = 10.0  
dt = 0.02     
steering_time = 5.0  
steering_start = 0.0
steering_target = ch.ChVector3d(0.5, 0, 0)  

while vis.GetTime() < t_end:
    if vis.GetTime() > steering_start and vis.GetTime() < steering_start + steering_time:
        steering = steering_target * (vis.GetTime() - steering_start) / steering_time
        driver.SetSteering(steering)
    
    system.DoStepDynamics(dt)
    vis.Render()
    irr.ChIrrApp.SoftwareShadowsOn()
    vis.UpdateCamera()

vis.Exit()