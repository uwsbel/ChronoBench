import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam Buckling Simulation")
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


beam_L = 10.0  
beam_wy = 1.0  
beam_wz = 1.0  
beam_E = 0.01e9  
beam_G = 0.01e9 * 0.3  
beam_rho = 1000  


mesh = fea.ChMesh()
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(beam_E)
msection.SetShearModulus(beam_G)
msection.SetRayleighDamping(0.000)


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))


mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)


mesh.AddElement(belement1)


sys.Add(mesh)


mfunY = chrono.ChFunctionSine(0.001, 1.5)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(mesh, chrono.ChFramed())
motor.SetMotionY(mfunY)


sys.Add(motor)


sys.SetSolverType(chrono.ChSolver.Type_MINRES)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)


vis.Initialize()
vis.AttachSystem(sys)


while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(0.01)


vis.GetSystem().Clear()