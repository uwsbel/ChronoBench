import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as mkl  
import pychrono.irrlicht as chronoirr  

print("Example: PyChrono using beam finite elements")  


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()



builder = fea.ChBuilderBeamEuler()
builder.SetLength(0.2 - (-0.1))  
builder.SetDirection(chrono.CH_DIRECTION_Y)  
builder.SetNElements(5)  
beam_section = builder.BuildBeam(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(-0.1, 0, -0.1))), 
                                 chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1))))  


beam_section.SetYoungModulus(0.01e9)  
beam_section.SetShearModulus(0.01e9 * 0.3)  
beam_section.SetRayleighDamping(0.000)  
beam_section.SetCentroid(0, -0.05)  
beam_section.SetShearCenter(0, -0.1)  
beam_section.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)  


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(-0.1, 0, -0.1)))  
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1)))  


mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)  
belement1.SetSection(beam_section)  
mesh.AddElement(belement1)  


last_beam_node = beam_section.GetLastBeamNodes().back()
last_beam_node.SetFixed(True)  


hnode1.SetForce(chrono.ChVector3d(0, -1, 0))


mtruss = chrono.ChBody()
mtruss.SetFixed(True)  
sys.Add(mtruss)  


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode2, mtruss, False, hnode2.Frame(), hnode2.Frame())
sys.Add(constr_bc)  
constr_bc.SetConstrainedCoords(True, True, True,  
                              True, True, True)  


constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
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