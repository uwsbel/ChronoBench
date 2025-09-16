import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as mkl  
import pychrono.irrlicht as chronoirr  

print("Example: PyChrono using beam finite elements")  


sys = chrono.ChSystemSMC()  

mesh = fea.ChMesh()  


msection = fea.ChBeamSectionEulerAdvanced()  

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)  

msection.SetYoungModulus(2.0e11)  
msection.SetShearModulus(8.0e10)  
msection.SetRayleighDamping(0.0)  
msection.SetSectionRotation(0 * chrono.CH_RAD_TO_DEG)  



builder = fea.ChBuilderBeamEuler(sys, msection)
builder.BuildBeam(chrono.ChVector3d(0, 0, -0.1), chrono.ChVector3d(0.2, 0, -0.1), 5, 'beam_1')


builder.GetFirstBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))


builder.GetLastBeamNodes().back().SetFixed(True)






mtruss = chrono.ChBody()
mtruss.SetFixed(True)  
sys.Add(mtruss)  
















mesh.SetAutomaticGravity(False)  


sys.Add(mesh)  


constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(builder.GetFirstBeamNodes().front(), mtruss, False, builder.GetFirstBeamNodes().front().Frame(),
                    builder.GetFirstBeamNodes().front().Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(True, True, True,  
                              True, True, True)  


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