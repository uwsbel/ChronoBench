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
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3) 
msection.SetRayleighDamping(0.000)


msection.SetCentroid(0, 0.02)  
msection.SetShearCenter(0, 0.1) 


msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD) 



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






mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr_bc = chrono.ChLinkMateGeneric()

constr_bc.Initialize(hnode3, mtruss, False, hnode3.GetAbsFrame(), hnode3.GetAbsFrame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,   
                               True, True, True)   


constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.GetAbsFrame(), hnode1.GetAbsFrame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,     
                              False, False, False)    






builder = fea.ChBuilderBeamEuler()



num_elements_beam1 = 5
beam1_ptA = chrono.ChVector3d(0, 0, -0.1)
beam1_ptB = chrono.ChVector3d(0.2, 0, -0.1)
beam1_Y_dir = chrono.ChVector3d(0, 1, 0)

builder.BuildBeam(mesh,
                  msection,
                  num_elements_beam1,
                  beam1_ptA,
                  beam1_ptB,
                  beam1_Y_dir)



first_builder_beam_nodes = builder.GetLastBeamNodes()
node_A_first_builder = first_builder_beam_nodes.front() 
node_B_first_builder = first_builder_beam_nodes.back()  



node_B_first_builder.SetFixed(True) 

node_A_first_builder.SetForce(chrono.ChVector3d(0, -1, 0)) 







start_node_for_new_beam = node_B_first_builder 
new_beam_endpoint_coords = chrono.ChVector3d(0.2, 0.1, -0.1)
new_beam_Y_up_direction = chrono.ChVector3d(0, 1, 0)
num_elements_for_new_beam = 5 

builder.BuildBeam(mesh,
                  msection,
                  num_elements_for_new_beam,
                  start_node_for_new_beam,    
                  new_beam_endpoint_coords,   
                  new_beam_Y_up_direction)






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

vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.3), chrono.ChVector3d(0.1, 0.05, -0.05))
vis.AddTypicalLights()




msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)