import pychrono as pc
import numpy as np


chrono = pc.ChSystemNSC()
assert chrono.Get_Gravity_x() == 0
assert chrono.Get_Gravity_y() == 0
assert chrono.Get_Gravity_z() == -9.81



ground = chrono.AddBody(pc.ChBodyEasyBox(10, 10, 0.1, 1000, 1000, 1000, True, True, True))
ground.SetPos(pc.ChVectorD(0, 0, -0.05))


turtlebot = chrono.AddBody(pc.ChBodyEasyCylinder(0.1, 0.15, 1000, 1000, 1000, True, True, True))
turtlebot.SetPos(pc.ChVectorD(0, 0, 0.075))
turtlebot.SetRot(pc.ChQuaternionD(1, 0, 0, 0))  


wheel1 = chrono.AddBody(pc.ChBodyEasyCylinder(0.02, 0.05, 1000, 1000, 1000, True, True, True))
wheel1.SetPos(pc.ChVectorD(0.075, 0.075, 0.075))
wheel1.SetRot(pc.ChQuaternionD(0, 0, 1, 0))  
wheel2 = chrono.AddBody(pc.ChBodyEasyCylinder(0.02, 0.05, 1000, 1000, 1000, True, True, True))
wheel2.SetPos(pc.ChVectorD(-0.075, 0.075, 0.075))
wheel2.SetRot(pc.ChQuaternionD(0, 0, 1, 0))  


chrono.AddJoint(pc.ChLinkRevolute(wheel1, turtlebot, pc.ChVectorD(0, 0, 0), pc.ChVectorD(0, 1, 0)))
chrono.AddJoint(pc.ChLinkRevolute(wheel2, turtlebot, pc.ChVectorD(0, 0, 0), pc.ChVectorD(0, 1, 0)))


motor1 = pc.ChLinkMotor(wheel1, True)
motor2 = pc.ChLinkMotor(wheel2, True)
chrono.AddLink(motor1)
chrono.AddLink(motor2)


chrono.Set_Gravity(pc.ChVectorD(0, 0, -9.81))


vis = pc.ChVisualSystemIrrlicht()
vis.Attach(chrono.GetSystem())
vis.SetWindowSize(800, 600)
vis.Initialize()
vis.AddCamera(pc.ChVectorD(1, 1, 1), pc.ChVectorD(0, 0, 0))
vis.AddLight(pc.ChVectorD(2, 2, 2), pc.ChColor(1, 1, 1), 10)


t = 0
dt = 0.01
while t < 10:
    
    if t > 1 and t < 3:
        motor1.SetTargetSpeed(1)
        motor2.SetTargetSpeed(-1)
    elif t > 5 and t < 7:
        motor1.SetTargetSpeed(-1)
        motor2.SetTargetSpeed(1)
    else:
        motor1.SetTargetSpeed(0)
        motor2.SetTargetSpeed(0)

    
    chrono.DoStepDt(dt)
    vis.Render()
    t += dt

vis.GetDevice().closeDevice()