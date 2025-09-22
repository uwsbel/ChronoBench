import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements with Euler-Bernoulli beam setup")


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
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)  


beam_L = 0.1


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))  
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))  


mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)


builder = fea.ChBuilderBeamEuler()
builder.SetMaterial(msection)
builder.SetCrossSectionType(beam_wy, beam_wz)
builder.SetLength(beam_L)
builder.SetFirstNode(hnode1)
builder.SetLastNode(hnode2)
beam1 = builder.BuildBeam()


builder2 = fea.ChBuilderBeamEuler()
builder2.SetMaterial(msection)
builder2.SetCrossSectionType(beam_wy, beam_wz)
builder2.SetLength(beam_L)
builder2.SetFirstNode(hnode2)
builder2.SetLastNode(hnode3)
beam2 = builder2.BuildBeam()


beam1.GetFirstNode().SetForce(chrono.ChVector3d(0, -1, 0))


mtruss = chrono.ChBody()
mtruss.SetFixed(True)  
sys.Add(mtruss)  


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(beam2.GetLastNode(), mtruss, False, beam2.GetLastNode().Frame(), beam2.GetLastNode().Frame())
sys.Add(constr_bc)  
constr_bc.SetConstrainedCoords(True, True, True,  
                              True, True, True)  


constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(beam1.GetFirstNode(), mtruss, False, beam1.GetFirstNode().Frame(), beam1.GetFirstNode().Frame())
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