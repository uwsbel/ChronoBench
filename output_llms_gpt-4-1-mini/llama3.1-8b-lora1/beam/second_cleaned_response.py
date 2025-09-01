import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


print("Example: PyChrono using beam finite elements")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()








builder = fea.ChBuilderBeamEuler()
beam = builder.SetYupDirection()
beam = builder.SetSection(msection)
beam = builder.SetLength(0.2)
beam = builder.SetNumElements(5)
beam = builder.SetFirstNode(chrono.ChFramed(chrono.ChVector3d(0, 0, -0.1)))
beam = builder.SetEndNode(chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1)))
beam = builder.BuildBeam()
beam.GetFirstBeamNodes().back().SetFixed(True)
beam.GetFirstBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))
mesh.AddElement(beam)


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0))) 
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam.GetLength(), 0, 0))) 
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam.GetLength() * 2, 0, 0))) 


mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)


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