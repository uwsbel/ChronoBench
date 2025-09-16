import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemFsi()


mesh = fea.ChMesh()


material = fea.ChMaterialBeamEulerAdvanced()
material.SetYoungModulus(0.01e9)
material.SetPoissonRatio(0.3)
material.SetRayleighDampingK(0.0)
material.SetRayleighDampingM(0.0)


beam = fea.ChElementBeamEuler()
beam.SetNodes(fea.ChNodeFEAxyzrot(), fea.ChNodeFEAxyzrot())


section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(0.1, 0.1)
section.SetArea(0.1*0.1)
section.SetIy(0.1*0.1*0.1*0.1/12)
section.SetIz(0.1*0.1*0.1*0.1/12)
section.SetJ(0.1*0.1*0.1*0.1/12 + 0.1*0.1*0.1*0.1/12)
beam.SetSection(section)


num_elements = 10
node_spacing = 1.0 / num_elements
for i in range(num_elements):
    node1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i*node_spacing, 0, 0)))
    node2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD((i+1)*node_spacing, 0, 0)))
    mesh.AddNode(node1)
    mesh.AddNode(node2)
    beam = fea.ChElementBeamEuler()
    beam.SetNodes(node1, node2)
    beam.SetMaterial(material)
    mesh.AddElement(beam)


mesh_container = fea.ChMeshContainer(mesh)
sys.Add(mesh_container)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


constraint = chrono.ChLinkMateGeneric()
constraint.Initialize(mesh.GetNode(0), ground, False, mesh.GetNode(0).GetFrame_C(), mesh.GetNode(0).GetFrame_C())
sys.Add(constraint)


motor_func = chrono.ChFunction_Ramp(0, 0.01)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(mesh.GetNode(num_elements), ground, chrono.ChFrameD(chrono.ChVectorD(num_elements, 0, 0)))
motor.SetAngleFunction(motor_func)
sys.Add(motor)


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, -3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()