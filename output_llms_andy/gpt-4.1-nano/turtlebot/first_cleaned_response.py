import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)



robot_body = chrono.ChBodyEasyBox(0.3, 0.2, 0.4, 1000, True, True)
robot_body.SetPos(chrono.ChVectorD(0, 0.1, 0))
robot_body.SetMass(2.0)
system.Add(robot_body)


wheel_radius = 0.05
wheel_width = 0.02

left_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 10, True, True)
right_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 10, True, True)


left_wheel_offset = chrono.ChVectorD(0, 0, 0.15)
right_wheel_offset = chrono.ChVectorD(0, 0, -0.15)

left_wheel.SetPos(robot_body.GetPos() + left_wheel_offset)
right_wheel.SetPos(robot_body.GetPos() + right_wheel_offset)


left_wheel.SetRot(chrono.Q_from_AngZ(0))
right_wheel.SetRot(chrono.Q_from_AngZ(0))

system.Add(left_wheel)
system.Add(right_wheel)


left_joint = chrono.ChLinkRevolute()
left_joint.Initialize(left_wheel, robot_body, chrono.ChCoordsysD(left_wheel.GetPos(), chrono.Q_from_AngX(0)))
system.Add(left_joint)

right_joint = chrono.ChLinkRevolute()
right_joint.Initialize(right_wheel, robot_body, chrono.ChCoordsysD(right_wheel.GetPos(), chrono.Q_from_AngX(0)))
system.Add(right_joint)



left_motor = chrono.ChLinkMotorRotationAngle()
right_motor = chrono.ChLinkMotorRotationAngle()


left_motor.Initialize(left_wheel, robot_body, chrono.ChCoordsysD(left_wheel.GetPos(), chrono.Q_from_AngX(0)))
right_motor.Initialize(right_wheel, robot_body, chrono.ChCoordsysD(right_wheel.GetPos(), chrono.Q_from_AngX(0)))

system.Add(left_motor)
system.Add(right_motor)



left_motor.SetAngleFunction(chrono.ChFunction_Const(0))
right_motor.SetAngleFunction(chrono.ChFunction_Const(0))


application = irr.ChIrrApp(system, "Turtlebot Simulation", irr.dimension2du(800, 600))
application.GetSceneManager().AddCamera(irr.ChCameraFPS())
application.GetSceneManager().AddLight(irr.ChLightSceneNode(irr.VECT(0, 10, 0), irr.VECT(1, 1, 1), 100))
application.AssetBindAll()
application.AssetUpdateAll()


application.GetCameraSceneNode().setPosition(chrono.ChVectorD(2, 2, 2))
application.GetCameraSceneNode().setTarget(chrono.ChVectorD(0, 0, 0))
application.GetLightSceneNode().setPosition(chrono.ChVectorD(0, 10, 0))


time_step = 0.005
total_time = 20  
current_time = 0



def get_motor_commands(t):
    
    left_speed = 0
    right_speed = 0

    if 0 <= t < 5:
        
        left_speed = 10  
        right_speed = 10
    elif 5 <= t < 8:
        
        left_speed = 10
        right_speed = -10
    elif 8 <= t < 15:
        
        left_speed = 10
        right_speed = 10
    elif 15 <= t < 18:
        
        left_speed = -10
        right_speed = 10
    else:
        
        left_speed = 0
        right_speed = 0
    return left_speed, right_speed


while application.GetDevice():
    
    left_speed, right_speed = get_motor_commands(current_time)

    
    
    left_motor.SetSpeed(chrono.ChFunction_Const(left_speed))
    right_motor.SetSpeed(chrono.ChFunction_Const(right_speed))

    
    system.DoStepDynamics(time_step)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    current_time += time_step