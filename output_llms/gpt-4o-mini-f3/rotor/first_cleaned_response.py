import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
from pychrono import irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

beam = fea.ChElementBeamANCF()
mesh.Add(beam)
beam.SetCoordinates([  
    chrono.ChVector3d(0, 0, 0),  
    chrono.ChVector3d(1, 0, 0),  
    chrono.ChVector3d(2, 0, 0)   
])
beam.SetNode(0, mesh.CreateNode())  
beam.SetNode(1, mesh.CreateNode())  
beam.SetNode(2, mesh.CreateNode())  
beam.SetMaterial(chrono.ChContactMaterialSMC())  


section = fea.ChBeamSectionEulerAdvanced()
section.SetDensity(100)  
section.SetYoungModulus(210e9)  
section.SetShearModulus(80e9)  
section.SetSectionWidth(0.1)  
section.SetSectionHeight(0.1)  
beam.SetSection(section)  


visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
visualize_beam.SetColorscaleMinMax(-400, 400)  
visualize_beam.SetSmoothFaces(True)  
visualize_beam.SetWireframe(False)  
mesh.AddVisualShapeFEA(visualize_beam)  


visualize_nodes = chrono.ChVisualShapeFEA(mesh)
visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
visualize_nodes.SetSymbolsThickness(0.005)  
visualize_nodes.SetSymbolsScale(0.01)  
visualize_nodes.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(visualize_nodes)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA beams')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(1, 0.5, 1.5))  
vis.AddTypicalLights()  


mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(False)  
sys.SetSolver(mkl_solver)  


time_step = 0.001


sys.Setup()
sys.Update()


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(time_step)