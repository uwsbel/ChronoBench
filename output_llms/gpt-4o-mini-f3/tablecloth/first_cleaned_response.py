import pychrono as chrono 
import pychrono.fea as fea 
import pychrono.pardisomkl as mkl 
import pychrono.irrlicht as chronoirr 

print("Example: folding a cloth with shell elements") 


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


mme = fea.ChMaterialShellKirchhoff() 
mme.SetDensity(100) 
mme.SetYoungModulus(1e5) 
mme.SetPoissonRatio(0.3) 


h = 0.01


n1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0))) 
n2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(1, 0, 0))) 
n3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(1, 1, 0))) 
n4 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 1, 0))) 


mesh.AddNode(n1)
mesh.AddNode(n2)
mesh.AddNode(n3)
mesh.AddNode(n4)


myelementA = fea.ChElementShellKirchhoff() 
myelementA.SetNodes(n1, n2, n3, n4) 
myelementA.SetMaterial(mme) 
myelementA.SetThickness(h) 


mesh.AddElement(myelementA)


force_mid = chrono.ChForce()
fapplication = chrono.ChForce()
fapplication.Force(chrono.ChVector3d(0, -50, 0), chrono.ChVector3d(0.5, 0, 0), True, force_mid)
mesh.AddForce(fapplication)


mvisualizeA = chrono.ChVisualShapeFEA(mesh)
mvisualizeA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ) 
mvisualizeA.SetColorscaleMinMax(-0.4, 0.4) 
mvisualizeA.SetSmoothFaces(True) 
mvisualizeA.SetWireframe(False) 
mesh.AddVisualizationShape(mvisualizeA) 


mvisualizeB = chrono.ChVisualShapeFEA(mesh)
mvisualizeB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS) 
mvisualizeB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) 
mvisualizeB.SetSymbolsThickness(0.006) 
mvisualizeB.SetSymbolsScale(0.01) 
mvisualizeB.SetZbufferHide(False) 
mesh.AddVisualizationShape(mvisualizeB) 


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Folding cloth with shell elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, 0.8), chrono.ChVector3d(0.5, 0, 0.5))
vis.AddTypicalLights()


msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver) 


while vis.Run():
    vis.BeginScene() 
    vis.Render() 
    vis.EndScene() 
    sys.DoStepDynamics(0.001)