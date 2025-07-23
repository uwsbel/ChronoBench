import pychrono as chrono   
import pychrono.fea as fea  
import pychrono.pardisomkl as mkl  
import pychrono.irrlicht as chronoirr  

print (chrono.GetChronoVersion())




sys = chrono.ChSystemSMC()





mesh = fea.ChMesh()





msection = fea.ChBeamSectionEulerAdvanced()

msection.SetDimension(beam_dA, beam_dI)  
msection.SetYoungModulus(young_modulus)  
msection.SetShearModulus(shear_modulus)  
msection.SetRayleighDamping(0.000)  
msection.SetCentroid(2, 0.2)  
msection.SetShearCenter(0, 0.3)  
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)  




beam_L = 0.5  
mesh.SetAutomaticGravity(False)  

for bp in range(3):
    
    msection271 = msection.Clone()
    msection271.SetSectionRotation(45 + bp * 90)  

    
    beam_fine = fea.ChBuilderBeamFEMCables()
    proef = beam_fine.BuildBeam(mesh,  
                                msection271,  
                                5,  
                                chrono.ChVector3d(0, 0, bp * 0.3),  
                                chrono.ChVector3d(1, 0, 0))  

    
    fea.ChBuilderBeamExtensionAARod().Build(mesh,  
                                            msection271,  
                                            2,  
                                            proef[1].GetPos() + chrono.ChVector3d(0, -0.2, 0),  
                                            chrono.ChVector3d(0, 1, 0))  




mtruss = chrono.ChBody()
mtruss.SetFixed(True)  
sys.Add(mtruss)  




msection.SetYoungModulus(0.01e9)  
msection.SetShearModulus(0.01e9 * 0.3)  
beam_L = 0.1  


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))  
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))  


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





mtruss = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
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





mesh.SetAutomaticGravity(False)

sys.Add(mesh)








sys.SetSolverType(chrono.ChSolver.Type_NULLSPACE)




while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.001)