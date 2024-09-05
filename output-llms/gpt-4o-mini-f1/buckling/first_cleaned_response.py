import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


section = fea.ChBeamSectionRectangular(0.02, 0.02)


material = fea.ChContinuumElastic()
material.SetE(30e6)  
material.Setnu(0.3)  


beamMaterial = fea.ChBeamSectionEulerAdvanced()
beamMaterial.SetDensity(7.85e-6)  
beamMaterial.SetGshear(0.5 * 30e6)  
beamMaterial.SetAsRectangularSection(0.02, 0.02)  
beamMaterial.SetIxx(0.0001)  
beamMaterial.SetIyy(0.0001)  
beamMaterial.SetJ(0.0001)  
beamMaterial.SetReferenceSection(0.02, 0.02)  


nodeA = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))
nodeB = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 1, 0), chrono.ChQuaterniond(1, 0, 0, 0))
nodeC = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 2, 0), chrono.ChQuaterniond(1, 0, 0, 0))
nodeD = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 3, 0), chrono.ChQuaterniond(1, 0, 0, 0))


mesh.AddNode(nodeA)
mesh.AddNode(nodeB)
mesh.AddNode(nodeC)
mesh.AddNode(nodeD)


elementAB = fea.ChElementBeamEuler()
elementAB.SetNodes(nodeA, nodeB)
elementAB.SetSection(beamMaterial)
mesh.AddElement(elementAB)

elementBC = fea.ChElementBeamEuler()
elementBC.SetNodes(nodeB, nodeC)
elementBC.SetSection(beamMaterial)
mesh.AddElement(elementBC)

elementCD = fea.ChElementBeamEuler()
elementCD.SetNodes(nodeC, nodeD)
elementCD.SetSection(beamMaterial)
mesh.AddElement(elementCD)


fixedNode = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))
mesh.AddNode(fixedNode)


constraint = chrono.ChLinkMateGeneric()
constraint.Initialize(nodeA, fixedNode, True, nodeA, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1))
sys.Add(constraint)


motor = chrono.ChLinkMotorRotationTorque()
motor.Initialize(nodeD, fixedNode, True)
sys.Add(motor)


class MyMotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    def Get_y(self, x):
        return 0.1 * math.sin(5 * x)

motor.SetMotorFunction(MyMotorFunction())


visualization = chrono.ChVisualShapeFEA(mesh)
visualization.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualization.SetColorscaleMinMax(-0.1, 0.1)
visualization.SetSmoothFaces(True)
visualization.SetWireframe(False)
mesh.AddVisualShapeFEA(visualization)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam buckling')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 5), chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()


sys.SetSolverType(chrono.ChSolver.Type_SOR)
sys.GetSolver().SetMaxIterations(50)
sys.SetTimestepperType(chrono.ChTimestepper.Type_ADAMS)


time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    time += 1e-3
    sys.DoStepDynamics(1e-3)