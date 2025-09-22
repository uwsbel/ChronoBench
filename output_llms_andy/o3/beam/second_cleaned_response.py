import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import math

print("Example: PyChrono using beam finite elements")




sys = chrono.ChSystemSMC()





mesh = fea.ChMesh()




msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)

msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.0)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)


msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)




beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(2 * beam_L, 0, 0)))

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


hnode2.SetForce(chrono.ChVectorD(4, 2, 0))
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))





mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False,
                     hnode3.Frame(), hnode3.Frame())
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_bc)



constr_a = chrono.ChLinkMateGeneric()
constr_a.Initialize(hnode1, mtruss, False,
                    hnode1.Frame(), hnode1.Frame())
constr_a.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_a)








builder = fea.ChBuilderBeamEuler()

point_A = chrono.ChVectorD(0, 0, -0.1)
point_B = chrono.ChVectorD(0.2, 0, -0.1)
up_dir  = chrono.ChVectorD(0, 1, 0)          


builder.BuildBeam(mesh,                       
                  msection,                   
                  5,                          
                  point_A,
                  point_B,
                  up_dir)


builder.GetLastBeamNodes()[-1].SetFixed(True)


builder.GetLastBeamNodes()[0].SetForce(chrono.ChVectorD(0, -1, 0))




mesh.SetAutomaticGravity(False)
sys.Add(mesh)




visualizebeamA = fea.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = fea.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams – Euler-Bernoulli example')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()




msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)




time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)