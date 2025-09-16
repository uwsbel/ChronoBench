import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


beam = fea.ChBeamSectionEuler()
beam.SetA(0.01)  
beam.SetIxx(1e-5)  
beam.SetIyy(1e-5)  
beam.SetJ(1e-6)  
beam.SetYoungModulus(2e11)  
beam.SetPoissonRatio(0.3)  
beam.SetDensity(7850)  


mesh = fea.ChBeamMesh(beam, 10, 0.1, 0.1, 0.1)
mesh.SetNodesPerElement(2)


beam_body = fea.ChBodyBeamMesh(mesh)
beam_body.SetPos(chrono.ChVectorD(0, 0, 0))
beam_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


def custom_motor(t):
    return 10 * np.sin(t)


motor = chrono.ChLinkMotorLinearSpeed()
motor.Initialize(beam_body, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(custom_motor)


sys.Add(beam_body)
sys.Add(ground)
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddCamera(chrono.ChVectorD(0, 5, 0))
vis.AddTypicalLights()


vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Simulation")
vis.SetLogo("")


solver = chrono.ChSolverNSC()
timestepper = chrono.ChTimestepperEuler()


sys.SetSolver(solver)
sys.SetTimestepper(timestepper)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()