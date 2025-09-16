import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine
import numpy as np


mysystem = chrono.ChSystemNSC()
mysystem.SetDt(0.01)


ground = chrono.ChBodyEasyBox(mysystem, 10, 2, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -3, 0))
ground.SetBodyFixed(True)
mysystem.Add(ground)


rover = chrono.ChBodyEasyBox(mysystem, 2, 1, 2, 100, True, True)
rover.SetPos(chrono.ChVectorD(0, 1, 0))
mysystem.Add(rover)


motor = chrono.ChMotorLinearTransmission()
motor.SetMaxForce(1000)
rover.AddMotor(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


vis.SetCameraUp(chrono.ChVectorD(0, 1, 0))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))


vis.AddTypicalLights()
vis.AddShadowAll()


vis.AddLogo(chrono.ChVectorD(0.5, 0.1, 0.1), chrono.ChVectorD(0.1, 0.1, 0.1), "chrono.png")


while vis.Run():
    mysystem.DoStepDynamics(mysystem.GetDt())
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()