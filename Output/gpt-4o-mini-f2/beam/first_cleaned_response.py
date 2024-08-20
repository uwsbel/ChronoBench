import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


nodeA = fea.ChNodeFEAxyz(sys, chrono.ChVector3d(0, 0.5, 0))
nodeB = fea.ChNodeFEAxyz(sys, chrono.ChVector3d(0, 1.5, 0))
nodeC = fea.ChNodeFEAxyz(sys, chrono.ChVector3d(0, 2.5, 0))
nodeD = fea.ChNodeFEAxyz(sys, chrono.ChVector3d(0, 3.5, 0))
nodeE = fea.ChNodeFEAxyz(sys, chrono.ChVector3d(0, 4.5, 0))
nodeF = fea.ChNodeFEAxyz(sys, chrono.ChVector3d(0, 5.5, 0))


mesh.AddNode(nodeA)
mesh.AddNode(nodeB)
mesh.AddNode(nodeC)
mesh.AddNode(nodeD)
mesh.AddNode(nodeE)
mesh.AddNode(nodeF)


beamAB = fea.ChElementBeamEuler()
beamAB.SetNodes(nodeA, nodeB)
beamAB.SetSection(fea.ChBeamSectionEulerAdvanced())
mesh.AddElement(beamAB)

beamBC = fea.ChElementBeamEuler()
beamBC.SetNodes(nodeB, nodeC)
beamBC.SetSection(fea.ChBeamSectionEulerAdvanced())
mesh.AddElement(beamBC)

beamCD = fea.ChElementBeamEuler()
beamCD.SetNodes(nodeC, nodeD)
beamCD.SetSection(fea.ChBeamSectionEulerAdvanced())
mesh.AddElement(beamCD)

beamDE = fea.ChElementBeamEuler()
beamDE.SetNodes(nodeD, nodeE)
beamDE.SetSection(fea.ChBeamSectionEulerAdvanced())
mesh.AddElement(beamDE)

beamEF = fea.ChElementBeamEuler()
beamEF.SetNodes(nodeE, nodeF)
beamEF.SetSection(fea.ChBeamSectionEulerAdvanced())
mesh.AddElement(beamEF)


sys.AddMesh(mesh)


vis_mesh = fea.ChVisualShapeFEA(mesh)
vis_mesh.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_mesh.SetColorscaleMinMax(-0.1, 0.1)
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -8))
vis.AddTypicalLights()


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIB)


time_step = 1e-3


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)