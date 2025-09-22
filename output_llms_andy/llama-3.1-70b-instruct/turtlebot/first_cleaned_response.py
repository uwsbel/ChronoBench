import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine
import numpy as np


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(sys, 10, 2, 10, 1000, True, chrono.ChVectorD(0, -2, 0), chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(ground)


turtlebot = chrono.ChBodyEasyCylinder(sys, 0.3, 0.1, 1000, True, chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(turtlebot)


left_motor = chrono.ChLinkMotorRotationSpeed()
left_motor.Initialize(turtlebot, ground, chrono.ChFrameD(chrono.ChVectorD(-0.15, 0.05, 0)))
sys.Add(left_motor)

right_motor = chrono.ChLinkMotorRotationSpeed()
right_motor.Initialize(turtlebot, ground, chrono.ChFrameD(chrono.ChVectorD(0.15, 0.05, 0)))
sys.Add(right_motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Turtlebot Simulation")
vis.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
vis.SetTypicalLenght(1.0)
vis.SetCameraUp(chrono.ChVectorD(0, 1, 0))
vis.SetCameraSpeed(0.5)
vis.SetCameraRotSpeed(0.1)
vis.AddTypicalLights()


tstep = 0.01
tend = 10
time = 0


while time < tend:
    
    if time < 2:
        left_motor.SetSpeed(-1)
        right_motor.SetSpeed(1)
    elif time < 4:
        left_motor.SetSpeed(1)
        right_motor.SetSpeed(-1)
    else:
        left_motor.SetSpeed(0)
        right_motor.SetSpeed(0)

    
    sys.DoStepDynamics(tstep)

    
    vis.Render()

    
    time += tstep


vis.Run()


vis.Finalize()