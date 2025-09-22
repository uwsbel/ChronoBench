import pychrono as chrono            
import pychrono.fea as fea           
import pychrono.pardisomkl as mkl    
import pychrono.irrlicht as chronoirr  

print("Example: PyChrono Euler–Bernoulli beam using ChBuilderBeamEuler")




sys = chrono.ChSystemSMC()




mesh = fea.ChMesh()




beam_wy = 0.012
beam_wz = 0.025
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)          
msection.SetShearModulus(0.01e9 * 0.3)    
msection.SetRayleighDamping(0.000)        
msection.SetCentroid(0, 0.02)             
msection.SetShearCenter(0, 0.1)           
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)




builder = fea.ChBuilderBeamEuler()

ptA = chrono.ChVectorD(0, 0, -0.1)
ptB = chrono.ChVectorD(0.2, 0, -0.1)
up  = chrono.ChVectorD(0, 1, 0)


builder.BuildBeam(mesh, msection,
                  5,
                  ptA, ptB,
                  True, True,
                  up)


first_nodes = builder.GetFirstBeamNodes()   
last_nodes  = builder.GetLastBeamNodes()







mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

constr_first = chrono.ChLinkMateGeneric()
constr_first.Initialize(first_nodes[0], mtruss,
                        False,
                        first_nodes[0].Frame(), first_nodes[0].Frame())
constr_first.SetConstrainedCoords(True, True, True,  
                                  True, True, True)  
sys.Add(constr_first)





last_nodes[-1].SetFixed(True)




first_nodes[0].SetForce(chrono.ChVector3d(0, -1, 0))




mesh.SetAutomaticGravity(False)
sys.Add(mesh)




vis_beam_mz = chrono.ChVisualShapeFEA(mesh)
vis_beam_mz.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_mz.SetColorscaleMinMax(-0.4, 0.4)
vis_beam_mz.SetSmoothFaces(True)
vis_beam_mz.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_mz)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Euler–Bernoulli Beam Example')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()




msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)




t_step = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(t_step)