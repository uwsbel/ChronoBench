import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()



builder = fea.ChBuilderBeamEuler()


point_A = chrono.ChVectorD(0, 0, -0.1)
point_B = chrono.ChVectorD(0.2, 0, -0.1)
builder.BuildBeam(mesh,                   
                 point_A, point_B,       
                  5,                      
                 chrono.ChVectorD(0, 1, 0)) 


builder.GetLastBeamNodes().back().SetFixed(True)


builder.GetLastBeamNodes()[0].SetForce(chrono.ChVectorD(0, -1, 0))



constr_a = chrono.ChLinkMateGeneric()
constr_a.Initialize(builder.GetLastBeamNodes()[0], sys.GetGroundBody(), False, builder.GetLastBeamNodes()[0].Frame(), builder.GetLastBeamNodes()[0].Frame())
sys.Add(constr_a)
constr_a.SetConstrainedCoords(True, True, True,  
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
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()


msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)