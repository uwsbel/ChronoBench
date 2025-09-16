import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math








system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


rover = chrono.ChChassis(
    "Viper", chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0),
    chrono.ChChassis.Type_VEHICLES, 1000
)
system.Add(rover)


driver = chrono.ChBodyEasySphere(0.2, 1000)
driver.SetPos(chrono.ChVectorD(0, 0.8, 0))
driver.SetBodyFixed(True)
system.Add(driver)


rover.AddSensor(driver)








vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()


vis.GetCamera().SetTarget(chrono.ChVectorD(0, 0.5, 0))
vis.GetCamera().SetPosition(chrono.ChVectorD(5, 2, 5))


vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChVectorD(0.8, 0.8, 0.8), 10)








timestep = 0.01


steering_start = 0
steering_end = math.pi / 4
steering_duration = 5


while vis.Run():
    
    simulation_time = system.GetChTime()
    steering_angle = chrono.ChInterp.Linear(
        steering_start, steering_end, simulation_time / steering_duration
    )

    
    

    
    system.DoStepDynamics(timestep)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()