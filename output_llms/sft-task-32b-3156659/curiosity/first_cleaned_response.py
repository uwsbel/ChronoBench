import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))



ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
system.Add(ground)


rover_chassis = chrono.ChBodyEasyBox(2, 0.5, 0.5, 1000, True, True)
rover_chassis.SetPos(chrono.ChVectorD(0, 0, -0.25))
rover_chassis.SetName("Rover Chassis")
rover_chassis.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
system.Add(rover_chassis)


wheel_left = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.25, 0.5, 1000, True, True)
wheel_left.SetPos(chrono.ChVectorD(-1, 0, -0.25))
wheel_left.SetName("Left Wheel")
wheel_left.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.3))
system.Add(wheel_left)

wheel_right = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.25, 0.5, 1000, True, True)
wheel_right.SetPos(chrono.ChVectorD(1, 0, -0.25))
wheel_right.SetName("Right Wheel")
wheel_right.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.3))
system.Add(wheel_right)


joint_left = chrono.ChLinkLockRevolute()
joint_left.Initialize(rover_chassis, wheel_left, chrono.ChFramed(chrono.ChVectorD(-1, 0, -0.25), chrono.Q_ROT_YAXIS))
system.AddLink(joint_left)

joint_right = chrono.ChLinkLockRevolute()
joint_right.Initialize(rover_chassis, wheel_right, chrono.ChFramed(chrono.ChVectorD(1, 0, -0.25), chrono.Q_ROT_YAXIS))
system.AddLink(joint_right)


motor_left = chrono.ChLinkMotorRotationSpeed()
motor_left.SetName("Left Wheel Motor")
motor_left.Initialize(wheel_left, rover_chassis, chrono.ChFramed(chrono.ChVectorD(-1, 0, -0.25)))
speed_func_left = chrono.ChFunctionConst(0)
motor_left.SetSpeedFunction(speed_func_left)
system.AddLink(motor_left)

motor_right = chrono.ChLinkMotorRotationSpeed()
motor_right.SetName("Right Wheel Motor")
motor_right.Initialize(wheel_right, rover_chassis, chrono.ChFramed(chrono.ChVectorD(1, 0, -0.25)))
speed_func_right = chrono.ChFunctionConst(0)
motor_right.SetSpeedFunction(speed_func_right)
system.AddLink(motor_right)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 728)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, 10), chrono.ChVectorD(0, 0, -0.25))
vis.AddTypicalLights(True, True)


time_step = 0.001
simulation_time = 0

while vis.Run() and simulation_time < 10:
    
    time = system.GetChTime()
    if time < 2:
        left_speed = 10.0
        right_speed = 10.0
    else:
        left_speed = 8.0
        right_speed = 12.0

    speed_func_left.SetConst(left_speed)
    speed_func_right.SetConst(right_speed)

    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    simulation_time += time_step