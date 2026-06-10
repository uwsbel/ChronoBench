import pychrono as chrono                
import pychrono.fea as fea               
import pychrono.pardisomkl as mkl        
import pychrono.irrlicht as chronoirr    

print("Example: PyChrono using beam finite elements")




sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()




beam_section = fea.ChBeamSectionEulerAdvanced()


beam_wy = 0.012
beam_wz = 0.025
beam_section.SetAsRectangularSection(beam_wy, beam_wz)


beam_section.SetYoungModulus(0.01e9)                
beam_section.SetShearModulus(0.01e9 * 0.3)          
beam_section.SetRayleighDamping(0.0)                


beam_section.SetCentroid(chrono.ChVector3d(0.0, 0.02, 0.0))
beam_section.SetShearCenter(chrono.ChVector3d(0.0, 0.1, 0.0))


beam_section.SetSectionRotation(45.0 * chrono.CH_DEG_TO_RAD)





print("\n--- Euler‑Bernoulli beam created with ChBuilderBeamEuler ---")


builder = fea.ChBuilderBeamEuler()


point_A = chrono.ChVector3d(0.0, 0.0, -0.1)
point_B = chrono.ChVector3d(0.2, 0.0, -0.1)


up_vec = chrono.ChVector3d(0.0, 1.0, 0.0)


num_elements = 5


builder.BuildBeam(mesh,                
                 beam_section,         
                 point_A, point_B,     
                 num_elements,         
                 up_vec)               


beam_nodes = builder.GetLastBeamNodes()
first_node = beam_nodes[0]          
last_node  = beam_nodes[-1]         


last_node.SetFixed(True)


first_node.SetForce(chrono.ChVector3d(0.0, -1.0, 0.0))

print(f"  Beam created: {num_elements} element(s) from {point_A} to {point_B}")
print(f"  First node id: {first_node.GetIndex()}, Last node id: {last_node.GetIndex()}")




beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0.0, 0.0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(2.0 * beam_L, 0.0, 0.0)))

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(beam_section)
mesh.AddElement(belement1)


belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(beam_section)
mesh.AddElement(belement2)


hnode2.SetForce(chrono.ChVector3d(4.0, 2.0, 0.0))      
hnode3.SetTorque(chrono.ChVector3d(0.0, -0.04, 0.0)) 





mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr_node3 = chrono.ChLinkMateGeneric()
constr_node3.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_node3)
constr_node3.SetConstrainedCoords(True, True, True,   
                                 True, True, True)    



constr_node1 = chrono.ChLinkMateGeneric()
constr_node1.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_node1)
constr_node1.SetConstrainedCoords(True, True, True,   
                                 True, True, True)    




mesh.SetAutomaticGravity(False)


sys.Add(mesh)





vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.4, 0.4)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)


vis_node = chrono.ChVisualShapeFEA(mesh)
vis_node.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_node.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_node.SetSymbolsThickness(0.006)
vis_node.SetSymbolsScale(0.01)
vis_node.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_node)




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