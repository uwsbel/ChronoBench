import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("data/")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


rover = chrono.ChWheeledVehicle()
rover.SetSystem(system)


chassis = chrono.ChBodyEasyBox(2, 1, 3, 100, True, True)
chassis.SetPos(chrono.ChVectorD(0, 1, 0))
rover.AddChassis(chassis)


wheel_radius = 0.5
wheel_width = 0.2
wheel_pos = [
    chrono.ChVectorD(1, 0, 1),
    chrono.ChVectorD(1, 0, -1),
    chrono.ChVectorD(-1, 0, 1),
    chrono.ChVectorD(-1, 0, -1),
]
for pos in wheel_pos:
    wheel = chrono.ChWheel(chrono.ChVectorD(0, -wheel_radius, 0), wheel_radius, wheel_width)
    wheel.SetFriction(0.8)
    wheel.SetSteering(True)
    rover.AddWheel(wheel, pos)




motor_driver = chrono.ChMotorControlDriver()
motor_driver.SetSteeringAngle(0)
motor_driver.SetThrottle(0)
rover.SetMotorControlDriver(motor_driver)




vis = chronoirr.ChIrrApp(system, "Curiosity Rover Simulation")
vis.SetWindowSize(1200, 800)
vis.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
vis.SetCameraLookAt(chrono.ChVectorD(0, 1, 0))


vis.AddCameraLight(chrono.ChVectorD(2, 5, 2))
vis.SetShadow(True)





while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    vis.DoStep()
    vis.EndScene()