import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


print("Example: PyChrono using Euler-Bernoulli beam finite elements")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


msection = fea.ChBeamSectionEuler()

msection.SetAsRectangularSection(0.01, 0.03)  


builder = fea.ChBuilderBeamEuler()


beam_length = 0.2
num_elements = 5
builder.SetMaterialProperties(msection)  
builder.SetLength(beam_length)  
builder.SetDirection(chrono.ChVector3d(0, 1, 0))  
builder.SetFirstCoordinate(chrono.ChVector3d(0, 0, -0.1))  
builder.SetLastCoordinate(chrono.ChVector3d(beam_length, 0, -0.1))  


beam = builder.BuildBeam()


mesh.AddElement(beam)







force_A = chrono.ChVector3d(0, -1, 0)
beam.GetNodes()[0].SetForce(force_A)



constraint_B = chrono.ChLinkMateGeneric()
constraint_B.Initialize(beam.GetNodes()[1], sys, False, beam.GetNodes()[1].Frame(), beam.GetNodes()[1].Frame())
constraint_B.SetConstrainedCoords(True, True, True)  


sys.Add(constraint_B)


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