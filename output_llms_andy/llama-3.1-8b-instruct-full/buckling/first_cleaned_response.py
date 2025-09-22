import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


L = 1
beam_wy = 0.012
beam_wz = 0.025
Iyy = (beam_wy * beam_wy) / 12.0
Izz = (beam_wz * beam_zz) / 12.0
A = beam_wy * beam_wz


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz, A, Iyy, Izz)
msection.SetEulerAdvanced()


h1 = 0
h2 = L
mnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(h1, 0, 0)))
mnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(h2, 0, 0)))


mesh.AddNode(mnode1)
mesh.AddNode(mnode2)


mbeam = fea.ChElementBeamEuler()
mbeam.AddNode(mnode1)
mbeam.AddNode(mnode2)
mbeam.SetSection(msection)
mesh.AddElement(mbeam)


mnode1.SetFixed(True)


mload = fea.ChLoadForce()
mload.SetFramed(chrono.ChFramed(chrono.ChVector3d(h2, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
mload.SetMagnitude(0.2)
mnode2.AddLoad(mload)


mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)


constr = chrono.ChLinkMateGeneric()
constr.Initialize(mnode2, mtruss, False, mnode2.Frame(), mnode2.Frame())
sys.Add(constr)
constr.SetConstrainedCoords(True, True, True, False, False, False)


mydriver = chrono.ChFunctionSine()
mydriver.SetFreq(0.1)
mydriver.SetPhase(0.2)


constr.SetDriverX(mydriver)
constr.SetDriverY(chrono.ChFunctionConstant(0))
constr.SetDriverZ(chrono.ChFunctionConstant(0))


mydriver2 = chrono.ChFunctionSine()
mydriver2.SetFreq(0.06)
mydriver2.SetPhase(0.2)


constr.SetDriverY(mydriver2)
constr.SetDriverX(chrono.ChFunctionConstant(0))
constr.SetDriverZ(chrono.ChFunctionConstant(0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beam buckling test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.0))
vis.AddTypicalLights()


log = chrono.ChLoggerSystem()
log.SetVerbose(True)
sys.AttachLogger(log)


mkl_solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(mkl_solver)


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)