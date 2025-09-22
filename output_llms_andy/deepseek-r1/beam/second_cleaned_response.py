import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as mkl  
import pychrono.irrlicht as chronoirr  

print("Example: PyChrono using Euler-Bernoulli beam finite elements")  


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


msection = fea.ChBeamSectionEulerAdvanced()


beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)  


msection.SetYoungModulus(0.01e9)  
msection.SetShearModulus(0.01e9 * 0.3)  
msection.SetRayleighDamping(0.000)  
msection.SetCentroid(0, 0.02)  
msection.SetShearCenter(0, 0.1)  
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  


mesh.SetAutomaticGravity(False)





builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,                   
    msection,               
    5,                      
    chrono.ChVector3d(0, 0, -0.1),  
    chrono.ChVector3d(0.2, 0, -0.1),  
    chrono.ChVector3d(0, 1, 0)  
)


first_node = builder.GetLastBeamNodes().front()
last_node = builder.GetLastBeamNodes().back()


first_node.SetForce(chrono.ChVector3d(0, -1, 0))



ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)


constr_end = chrono.ChLinkMateGeneric()
constr_end.Initialize(last_node, ground, False, last_node.Frame(), last_node.Frame())
sys.Add(constr_end)
constr_end.SetConstrainedCoords(True, True, True, True, True, True)  


sys.Add(mesh)


visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
visualizebeamA.SetSmoothFaces(True)  
visualizebeamA.SetWireframe(False)  
mesh.AddVisualShapeFEA(visualizebeamA)


visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
visualizebeamC.SetSymbolsThickness(0.006)  
visualizebeamC.SetSymbolsScale(0.01)  
visualizebeamC.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(visualizebeamC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA beams')  
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