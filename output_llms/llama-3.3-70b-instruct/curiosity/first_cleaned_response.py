import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mkl as chronomkl
import numpy as np


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.5)
contact_material.SetCompliance(1e-6)
contact_material.SetComplianceT(1e-6)


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)


rover = chrono.ChBodyEasyBox(sys, 1, 1, 2, 100, True, True, contact_material)
rover.SetPos(chrono.ChVectorD(0, 1, 0))
rover.SetMass(100)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rover, sys.GetGroundBody(), chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.Q_from_AngX(0)))
motor.SetSpeed(1)


sys.Add(ground)
sys.Add(rover)
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddCamera(chrono.ChVectorD(0, 5, 10))
vis.SetWindowSize(1024, 768)
vis.SetLogo("")


vis.AddTypicalGround(chrono.ChVectorD(0, -1, 0))
vis.AddTypicalGroundLogo()


time_step = 0.01
while sys.GetChTime() < 10:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.DoStep()


vis.CloseWindow()