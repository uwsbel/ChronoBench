import pychrono as chrono
from pychrono import irrlicht
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBodyEasyBox(100, 100, 0.1, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetFixed(True)
system.Add(ground)


chassis = chrono.ChBodyEasyBox(0.2, 0.1, 0.05, 100, True, False)
chassis.SetPos(chrono.ChVectorD(0, 0.05, 0))
chassis.SetFixed(False)
system.Add(chassis)


left_wheel = chrono.ChBodyEasyCylinder(0.05, 0.05, 100, True, False)
left_wheel.SetPos(chrono.ChVectorD(-0.1, 0, 0))
left_wheel.SetFixed(False)
system.Add(left_wheel)

right_wheel = chrono.ChBodyEasyCylinder(0.05, 0.05, 100, True, False)
right_wheel.SetPos(chrono.ChVectorD(0.1, 0, 0))
right_wheel.SetFixed(False)
system.Add(right_wheel)


joint_left = chrono.ChLinkRevolute()
joint_left.Initialize(chassis, left_wheel, chrono.ChFrameD(chrono.ChVectorD(-0.1, 0, 0), chrono.Q_from_AngZ(0)))
system.Add(joint_left)

joint_right = chrono.ChLinkRevolute()
joint_right.Initialize(chassis, right_wheel, chrono.ChFrameD(chrono.ChVectorD(0.1, 0, 0), chrono.Q_from_AngZ(0)))
system.Add(joint_right)


motor_left = chrono.ChLinkMotorRevolute()
motor_left.Initialize(joint_left, chrono.ChFunctionConst(0), chrono.ChFunctionConst(0))
system.Add(motor_left)

motor_right = chrono.ChLinkMotorRevolute()
motor_right.Initialize(joint_right, chrono.ChFunctionConst(0), chrono.ChFunctionConst(0))
system.Add(motor_right)


app = irrlicht.ChIrrApp(system, "Turtlebot Simulation", irrlicht.dimension2d_i(800, 600))
app.AddCamera(chrono.ChVectorD(0, 1, 2), chrono.ChVectorD(0, 0, 0))
app.AddLight(chrono.ChVectorD(0, 10, 10), 10)


time_step = 1.0 / 60.0
while app.GetDevice().run():
    current_time = system.GetChTime()
    
    if current_time < 5:
        left_torque = 10
        right_torque = -10
    elif current_time < 10:
        left_torque = -10
        right_torque = 10
    else:
        left_torque = 0
        right_torque = 0
    motor_left.SetTorque(left_torque)
    motor_right.SetTorque(right_torque)
    system.DoStepDynamics(time_step)
    app.BeginScene()
    app.DrawAll()
    app.EndScene()