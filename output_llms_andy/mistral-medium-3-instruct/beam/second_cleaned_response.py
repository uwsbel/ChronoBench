import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()





builder = fea.ChBuilderBeamEuler()
builder.SetBeamSection(fea.ChBeamSectionEulerAdvanced())
builder.GetLastBeamSection().SetAsRectangularSection(0.012, 0.025)
builder.GetLastBeamSection().SetYoungModulus(0.01e9)
builder.GetLastBeamSection().SetShearModulus(0.01e9 * 0.3)
builder.GetLastBeamSection().SetRayleighDamping(0.000)
builder.GetLastBeamSection().SetCentroid(0, 0.02)
builder.GetLastBeamSection().SetShearCenter(0, 0.1)
builder.GetLastBeamSection().SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)


beam = builder.BuildBeam(mesh,
                        chrono.ChVector3d(0, 0, -0.1),
                        chrono.ChVector3d(0.2, 0, -0.1),
                        5,
                        chrono.ChVector3d(0, 1, 0))  





last_node = builder.GetLastBeamNodes().back()
last_node.SetFixed(True)


first_node = builder.GetLastBeamNodes().front()
first_node.SetForce(chrono.ChVector3d(0, -1, 0))







mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(first_node, mtruss, False, first_node.Frame(), first_node.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,  
                              False, False, False)  


mesh.SetAutomaticGravity(False)


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