import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import pychrono.sensor as sens


sys = chrono.ChSystemNSC()


mesh = fea.ChMesh()


node1 = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
node2 = fea.ChNodeFEAxyzrot(chrono.ChVector3d(1, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
node3 = fea.ChNodeFEAxyzrot(chrono.ChVector3d(2, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
node4 = fea.ChNodeFEAxyzrot(chrono.ChVector3d(3, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))


mesh.AddNode(node1)
mesh.AddNode(node2)
mesh.AddNode(node3)
mesh.AddNode(node4)


beamelem1 = fea.ChElementBeamEuler()
beamelem1.SetNodes(node1, node2)
beamelem1.SetAsRectangularSection(0.020, 0.030)
beamelem1.SetYoungModulus(0.01e9)

beamelem2 = fea.ChElementBeamEuler()
beamelem2.SetNodes(node2, node3)
beamelem2.SetAsRectangularSection(0.020, 0.030)
beamelem2.SetYoungModulus(0.01e9)

beamelem3 = fea.ChElementBeamEuler()
beamelem3.SetNodes(node3, node4)
beamelem3.SetAsRectangularSection(0.020, 0.030)
beamelem3.SetYoungModulus(0.01e9)


mesh.AddElement(beamelem1)
mesh.AddElement(beamelem2)
mesh.AddElement(beamelem3)


beam_model = fea.ChBeamFEA(mesh)


node1.SetInitialDisplacement(chrono.ChVector3d(0, 0, 0.1))


node1.SetFixed(True)


sys.Add(beam_model)


solver = chrono.ChSolverPSOR()


sys.SetSolver(solver)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2,2,2),chrono.ChVector3d(2,0,0))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)