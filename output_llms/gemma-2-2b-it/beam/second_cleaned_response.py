import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardiso as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using Euler-Bernoulli beams")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


builder = fea.ChBuilderBeamEuler()
builder.SetNodes(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)), chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1)))  
builder.SetSection(fea.ChBeamSectionEulerAdvanced())
builder.SetYUp(True)
builder.SetElements(5)  
beam = builder.BuildBeam()


builder.GetLastBeamNodes().back().SetFixed(True)


beam.GetFirstNode().SetForce(chrono.ChVector3d(0, -1, 0))


mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(beam.GetFirstNode(), mtruss, False, beam.GetFirstNode().Frame(), beam.GetFirstNode().Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True) 
constr_d.SetConstrainedCoords(False, False, False) 


mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(beam.GetLastNode(), mtruss, False, beam.GetLastNode().Frame(), beam.GetLastNode().Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True) 
constr_bc.SetConstrainedCoords(False, False, False) 


mesh.SetAutomaticGravity(False)


sys.Add(mesh)


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