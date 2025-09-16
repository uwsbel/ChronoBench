import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()



msection = fea.ChBeamSectionEulerBernoulli()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)


msection.SetYoungModulus(0.01e9)  
msection.SetShearModulus(0.01e9 * 0.3)  
msection.SetRayleighDamping(0.000)  


builder = fea.ChBuilderBeamEuler()
builder.InitSection(msection)
builder.SetNodesSpacing(0.04)  
builder.SetNofElements(5)  


points_A = chrono.ChVector3d(0, 0, -0.1)
points_B = chrono.ChVector3d(0.2, 0, -0.1)
up_dir = chrono.ChVector3d(0, 1, 0)  
builder.BuildBeam(mesh, points_A, points_B, up_dir)


last_node = builder.GetLastBeamNodes().back()
last_node.SetFixed(True)


first_node = builder.GetBeamNodes().front()
first_node.SetForce(chrono.ChVector3d(0, -1, 0))


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


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)