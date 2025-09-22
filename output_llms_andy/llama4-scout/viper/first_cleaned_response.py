import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh


chrono.ChEngine.Initialize()


system = chrono.ChSystemNSC()


system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, -1, 0))
ground.SetFixed(True)
system.AddBody(ground)


viper = veh.ChVehicle(veh.VehicleType.Viper)
system.Add(viper.GetChassisBody())


viper.Initialize(system)


driver = veh.ChDriver(viper)
driver.SetSpeed(5)  
system.Add(driver.GetSteering())


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(chrono.ChVector2i(800, 600))
vis.SetWindowTitle("Viper Rover Simulation")
vis.EnableSFX(True)
vis.EnableShadows(True)


camera = vis.GetCamera()
camera.SetPosition(chrono.ChVector3d(10, 10, 10))
camera.LookAt(chrono.ChVector3d(0, 0, 0))


timestep = 0.01
total_time = 10.0
steering_change_time = 5.0
initial_steering = 0.0
final_steering = 0.5


t = 0.0
while t < total_time:
    
    if t < steering_change_time:
        steering = initial_steering + (final_steering - initial_steering) * t / steering_change_time
    else:
        steering = final_steering

    driver.SetSteering(steering)

    
    system.DoStepDynamics(timestep)

    
    vis.Render()

    
    t += timestep


vis.Exit()
chrono.ChEngine.Finalize()