import pychrono as ch
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


sys = ch.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangular(0.1, 0.01, 210e9, 0.01, 0.01, 7800)  
section.UpdateGeometry()


iga_beam = fea.ChBeamEulerAdvanced(section, 30)  
iga_beam.SetDelta(0.001)  
mesh.AddBeamEulerAdvanced(iga_beam)  


pA = ch.ChVector3d(-2, 0, 0)
pB = ch.ChVector3d(2, 0, 0)


iga_beam.SetCoords(pA, pB)


body = ch.ChBodyEasyCylinder(0.5, 0.1, 100, True, True)  
body.SetPos(iga_beam.GetPointCoord(0.5))  
body.SetFixed(False)  
sys.AddBody(body)  


truss = fea.ChElementBeamEuler()
truss.SetSection(section)  
truss.SetNodes(iga_beam.GetNode(iga_beam.GetNnodes() // 2), fea.ChNodeFEAxyz(body))  
mesh.AddElement(truss)  


motor = ch.ChLinkMotorRotationSpeed()
motor.Initialize(iga_beam.GetNode(0), iga_beam.GetNode(0))  
motor.SetSpeedFunction(ch.ChFunctionConst(30))  
sys.Add(motor)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott rotor with IGA beam')
vis.Initialize()
vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(ch.ChVector3d(0, 0.5, -2))
vis.AddTypicalLights()


mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.SetVerbose(True)
mkl_solver.SetMaxIterations(100)
sys.SetSolver(mkl_solver)


time_step = 0.001


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)