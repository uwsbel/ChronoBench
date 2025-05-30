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
msection.SetRayleighDamping(0.0)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)

msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)




beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot()
hnode1.SetPos(chrono.ChVector3d(0, 0, 0))
hnode1.SetRot(chrono.QUNIT)
mesh.AddNode(hnode1)

hnode2 = fea.ChNodeFEAxyzrot()
hnode2.SetPos(chrono.ChVector3d(beam_L, 0, 0))
hnode2.SetRot(chrono.QUNIT)
mesh.AddNode(hnode2)

hnode3 = fea.ChNodeFEAxyzrot()
hnode3.SetPos(chrono.ChVector3d(beam_L * 2, 0, 0))
hnode3.SetRot(chrono.QUNIT)
mesh.AddNode(hnode3)


be1 = fea.ChElementBeamEuler()
be1.SetNodes(hnode1, hnode2)
be1.SetSection(msection)
mesh.AddElement(be1)


be2 = fea.ChElementBeamEuler()
be2.SetNodes(hnode2, hnode3)
be2.SetSection(msection)
mesh.AddElement(be2)


hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))





mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr1 = chrono.ChLinkMateGeneric()
constr1.Initialize(hnode3, mtruss, False,
                   hnode3.GetFrame(), mtruss.GetFrame())
constr1.SetConstrainedCoords(True, True, True,   
                             True, True, True)   
sys.Add(constr1)


constr2 = chrono.ChLinkMateGeneric()
constr2.Initialize(hnode1, mtruss, False,
                   hnode1.GetFrame(), mtruss.GetFrame())
constr2.SetConstrainedCoords(False, True, True,   
                             False, False, False) 
sys.Add(constr2)





builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, msection, 5,
                  chrono.ChVector3d(0, 0, -0.1),
                  chrono.ChVector3d(0.2, 0, -0.1),
                  chrono.ChVector3d(0, 1, 0))


last_beam_nodes = builder.GetLastBeamNodes()
last_beam_nodes[-1].SetFixed(True)
last_beam_nodes[0].SetForce(chrono.ChVector3d(0, -1, 0))






A_node = last_beam_nodes[-1]
builder.BuildBeam(mesh, msection, 5,
                  A_node,
                  chrono.ChVector3d(0.2, 0.1, -0.1),
                  chrono.ChVector3d(0, 1, 0))




mesh.SetAutomaticGravity(False)
sys.Add(mesh)


vis_shape = chrono.ChVisualShapeFEA(mesh)
vis_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_shape.SetColorscaleMinMax(-0.4, 0.4)
vis_shape.SetSmoothFaces(True)
vis_shape.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_shape)


node_csys = chrono.ChVisualShapeFEA(mesh)
node_csys.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
node_csys.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
node_csys.SetSymbolsThickness(0.006)
node_csys.SetSymbolsScale(0.01)
node_csys.SetZbufferHide(False)
mesh.AddVisualShapeFEA(node_csys)




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




tstep = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(tstep)