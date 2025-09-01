import numpy as np
from pychrono.core import ChSystemNSC
from pychrono.fea import ChMesh, ChBeamSectionEuler, ChElementBeamEuler, ChNodeFEAxyzrot
from pychrono.engine import ChEngine
from pychrono.irrlicht import ChVisualSystemIrrlicht
from pychrono import ChFunction


sys = ChSystemNSC()


mesh = ChMesh()
msection = ChBeamSectionEuler()
msection.SetAsRectangularSection(0.01, 0.01)  
msection.SetYoungModulus(2e11)  
msection.SetShearModulus(7.692e10)
msection.SetRayleighDamping(0.000)
hnode1 = ChNodeFEAxyzrot(ChFramed(ChVectorD(0, 0, 0)))
hnode2 = ChNodeFEAxyzrot(ChFramed(ChVectorD(1, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
belement1 = ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)


sys.Add(mesh)


flywheel = ChBodyEasySphere(0.05, 7850, True, True)
flywheel.SetPos(ChVectorD(0.5, 0, 0))
sys.Add(flywheel)


motor = ChLinkMotorRotationSpeed()
motor.Initialize(mesh, flywheel, ChCoordsysD(ChVectorD(0.5, 0, 0), QUNIT))
sys.Add(motor)
motor.SetSpeedFunction(ChFunction_Const(10.0))  


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(ChVectorD(1, 1, 1))
vis.AddTypicalLights()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()