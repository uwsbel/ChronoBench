import pychrono as chrono
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)



ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


rover = chrono.ChWheeledVehicle()
rover.SetSystem(my_system)


chassis = chrono.ChBodyEasyBox(2, 1, 3, 1000, True, True)
chassis.SetPos(chrono.ChVectorD(0, 1, 0))
rover.AddChassis(chassis)


wheel_radius = 0.3
wheel_width = 0.2
for i in range(6):
    wheel = chrono.ChWheel(chrono.ChVectorD(0, 0, 0), wheel_radius, wheel_width)
    wheel.SetFrictionCoefficient(0.8)
    wheel.SetSuspensionForce(1000)
    wheel.SetSpringCoefficient(1000)
    wheel.SetDampingCoefficient(100)
    rover.AddWheel(wheel)




motor_driver = chrono.ChMotorDriver()
motor_driver.SetTargetSpeed(1)
rover.SetMotorDriver(motor_driver)


vis = chronoirr.ChIrrApp(my_system, 'Curiosity Rover Simulation')
vis.SetWindowSize(1280, 720)
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))



vis.AssetBindAll()
vis.SetShowInfos(True)
vis.SetPaused(False)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.DoStep()
    vis.EndScene()