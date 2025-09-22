import os
import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/data')  
sys = chrono.ChSystemSMC()  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(5, 5, 5))
vis.AddSkyBox()


mesh = fea.ChMesh()


beam_length = 5.0  
beam_height = 0.1  
beam_width = 0.1   
density = 1000     
E = 210e9         
nu = 0.3          


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_width, beam_height)
msection.SetYoungModulus(E)
msection.SetShearModulus(E / (2 * (1 + nu)))  
msection.SetRayleighDamping(0.01)  


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)


sys.Add(mesh)


fixed_support = chrono.ChLinkLock()
fixed_support.Initialize(hnode1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.Add(fixed_support)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(hnode1, hnode2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(0.0))  
sys.Add(motor)


def apply_buckling_force(time):
    if time < 1.0:  
        motor.SetSpeedFunction(chrono.ChFunction_Const(2.0 * math.pi))  
    else:
        motor.SetSpeedFunction(chrono.ChFunction_Const(0.0))  


time_step = 0.01
solver = chrono.ChSolverPSOR()  
sys.SetSolver(solver)
sys.SetMaxItersSolverSpeed(50)  
sys.SetMaxItersSolverStab(50)   


while vis.Run():
    
    apply_buckling_force(sys.GetChTime())
    
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.Close()