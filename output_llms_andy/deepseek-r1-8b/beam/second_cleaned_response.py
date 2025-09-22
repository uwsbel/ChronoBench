import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


print("Example: PyChrono using Euler-Bernoulli beam finite elements")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()



beam_builder = fea.ChBuilderBeamEuler()
beam_builder.SetLength(0.2)  
beam_builder.SetDirection(chrono.ChVector3d(0, 0, -1))  
beam_builder.SetYoungModulus(0.01e9)  
beam_builder.SetNumberOfElements(5)  
beam_builder.SetFirstYDirection(chrono.ChVector3d(0, 1, 0))  
beam_builder.SetSectionRotation(0)  


beam = beam_builder.BuildBeam()



node_A = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, -0.1)))

node_B = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1)))


mesh.AddNode(node_A)
mesh.AddNode(node_B)


beam_element = fea.ChElementBeamEuler()
beam_element.SetNodes(node_A, node_B)
beam_element.SetSection(beam_builder.GetSection())
mesh.AddElement(beam_element)


constraint_B = chrono.ChLinkMateGeneric()
constraint_B.Initialize(node_B, sys.GetFixedBody(), False, node_B.Frame(), node_B.Frame())
constraint_B.SetConstrainedCoords(True, True, True)
sys.Add(constraint_B)


node_A.SetForce(chrono.ChVector3d(0, -1, 0))


visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
visualize_beam.SetSmoothFaces(True)
visualize_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(visualize_beam)


visualize_nodes = chrono.ChVisualShapeFEA(mesh)
visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualize_nodes.SetSymbolsThickness(0.006)
visualize_nodes.SetSymbolsScale(0.01)
visualize_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualize_nodes)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Euler-Bernoulli Beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()


msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)