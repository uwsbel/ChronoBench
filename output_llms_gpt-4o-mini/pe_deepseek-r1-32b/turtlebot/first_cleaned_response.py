import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3D(0, 3, 10))
vis.AddTypicalLights()


ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3D(0, 0, 0))
ground.SetMass(0)
ground.SetFixed(True)
ground.EnableCollision(True)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3D(10, 10, 1))
ground.AddVisualShape(ground_shape)
system.Add(ground)


turtlebot = chrono.ChBody()
turtlebot.SetPos(chrono.ChVector3D(0, 0, 0.5))  
turtlebot.SetMass(1.0)
turtlebot.SetInertiaXX(chrono.ChVector3D(0.1, 0.1, 0.1))
system.Add(turtlebot)


wheel_left = chrono.ChBody()
wheel_left.SetPos(chrono.ChVector3D(-0.1, 0, 0))
wheel_left.SetMass(0.1)
wheel_left.SetInertiaXX(chrono.ChVector3D(0.01, 0.01, 0.01))
system.Add(wheel_left)

wheel_right = chrono.ChBody()
wheel_right.SetPos(chrono.ChVector3D(0.1, 0, 0))
wheel_right.SetMass(0.1)
wheel_right.SetInertiaXX(chrono.ChVector3D(0.01, 0.01, 0.01))
system.Add(wheel_right)


joint_left = chrono.ChLinkLockRevolute()
joint_left.Initialize(turtlebot, wheel_left, chrono.ChCoordsysD(chrono.ChVector3D(-0.1, 0, 0), chrono.QUNIT))
system.Add(joint_left)

joint_right = chrono.ChLinkLockRevolute()
joint_right.Initialize(turtlebot, wheel_right, chrono.ChCoordsysD(chrono.ChVector3D(0.1, 0, 0), chrono.QUNIT))
system.Add(joint_right)


motor_left = chrono.ChLinkMotorRotationSpeed()
motor_left.Initialize(turtlebot, wheel_left, chrono.ChCoordsysD(chrono.ChVector3D(-0.1, 0, 0), chrono.QUNIT))
system.Add(motor_left)

motor_right = chrono.ChLinkMotorRotationSpeed()
motor_right.Initialize(turtlebot, wheel_right, chrono.ChCoordsysD(chrono.ChVector3D(0.1, 0, 0), chrono.QUNIT))
system.Add(motor_right)


turtlebot_shape = chrono.ChVisualShapeBox(chrono.ChVector3D(0.2, 0.2, 0.2))
turtlebot.AddVisualShape(turtlebot_shape)

wheel_shape = chrono.ChVisualShapeCylinder(0.05, 0.1)
wheel_left.AddVisualShape(wheel_shape)
wheel_right.AddVisualShape(wheel_shape)


time_step = 0.01
simulation_time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    current_time = simulation_time
    if 1 <= current_time < 2:
        
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(-1.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(1.0))
    elif 2 <= current_time < 3:
        
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(1.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(-1.0))
    else:
        
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(1.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(1.0))
    
    system.DoStepDynamics(time_step)
    simulation_time += time_step


vis.Close()