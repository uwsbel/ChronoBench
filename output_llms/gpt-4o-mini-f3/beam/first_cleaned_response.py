import pychrono as chrono 
import pychrono.fea as fea 
import pychrono.pardisomkl as mkl 
import pychrono.irrlicht as chronoirr 

print("Example: PyChrono using triangular beam finite elements") 


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


msection = fea.ChBeamSectionEulerAdvanced()


beam_diameter = 0.012
msection.SetAsRectangularSection(beam_diameter, beam_diameter) 


msection.SetYoungModulus(0.01e9) 
msection.SetShearModulus(0.01e9 * 0.3) 
msection.SetRayleighDamping(0.000) 
msection.SetCentroid(0, 0) 
msection.SetShearCentre(0, 0) 
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG) 


beam_length = 0.05


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 0)) 
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChVector3d(beam_length, 0, 0)) 
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChVector3d(beam_length * 2, 0, 0)) 


mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2) 
belement1.SetSection(msection) 
mesh.AddElement(belement1) 


belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3) 
belement2.SetSection(msection) 
mesh.AddElement(belement2) 


hnode2.SetForce(chrono.ChVector3d(4, 2, 0))


hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))


mtruss = chrono.ChBody()
mtruss.SetFixed(True) 
sys.Add(mtruss) 


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
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