import pychrono as chrono
import numpy as np


sim = chrono.ChSimulation()
sys = chrono.ChSystemSMC()
sys.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
sim.AddSystem(sys)


beam = chrono.ChBeam()
beam.SetMaterial(chrono.ChMaterialSurfaceNSC())
beam.SetSection(chrono.ChBeamSectionFrame(chrono.ChFrame(), 0.01, 0.01))
beam.SetLength(1)
beam.SetYieldStrength(1e6)
beam.SetYoungModulus(2e11)
beam.SetPoissonRatio(0.3)
beam.SetNumberOfElements(10)
beam.Set_Frame_Follower(True)
sys.Add(beam)


motor = chrono.ChLinkMotor()
motor.Initialize(beam, chrono.ChFrameD(chrono.ChVectorD(1, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)), chrono.ChVectorD(1, 0, 0))
motor.SetPos(0)
motor.SetForce(100)
sys.Add(motor)


fixed = chrono.ChLinkLock()
fixed.Initialize(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(fixed)


sys.SetSolver(chrono.ChSolverSMC())
sys.SetTimestepper(chrono.ChTimestepperHHT())
sys.SetSolverMaxIterations(100)
sys.SetSolverTolerance(1e-6)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.Initialize()
vis.AddLogo(chrono.ChVectorD(0, 0, 0))


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()