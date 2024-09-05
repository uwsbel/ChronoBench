import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  


mesh = chrono.ChVisualShapeTriangleMesh()
mesh.SetMeshFilename(chrono.GetChronoDataFile("vehicles/meshes/beam_triangles.obj"))
mesh.SetName("beam_mesh")


fem_manager = chrono.ChFEMManager(sys)


container = chrono.ChElementHexaTetraContainer()
fem_manager.AddLayer(container)


matFEA = chrono.ChContinuumElastic()
matFEA.SetYoungModulus(2e11)  
matFEA.SetPoissonRatio(0.3)   
matFEA.SetDensity(7800)       


section = chrono.ChBeamSectionEulerAdvanced()
section.Initialize(container.GetBeamGrid())
section.SetDensity(7800)  
section.SetYoungModulus(2e11)  
section.SetShearModulus(8e10)  
section.SetSection3DXY(0.01, 0.02, 0.0, 0.0, 0.0)  
section.SetJ(1e-4)  
section.SetThetaA(0)  
section.SetThetaB(0)  
section.SetWarpingConstant(1e-6)  
container.AddBeamSection(section, matFEA)  


meshELEM = chrono.ChVisualShapeTriangleMesh()
meshELEM.SetMeshFilename(chrono.GetChronoDataFile("vehicles/meshes/beam_elem_triangles.obj"))
meshELEM.SetName("elem_mesh")
container.GetBeamGrid().GetChBody().AddVisualShape(meshELEM)


ground = chrono.ChBody()
sys.AddBody(ground)


truss = chrono.ChBody()
sys.AddBody(truss)


constr = chrono.ChLinkLock()
constr.Initialize(truss, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(constr)


motor_fun = chrono.ChFunctionConst(0)
truss.AddForce(chrono.ChForceTorqueBody(motor_fun, chrono.ChVector3d(0, 0, 0)))


meshed_beam = chrono.ChBody()
sys.AddBody(meshed_beam)
meshed_beam.AddVisualShape(mesh)


frame_beam = chrono.ChFramed(chrono.ChVector3d(0, 0, 0))


constr2 = chrono.ChLinkMateGeneric()
constr2.Initialize(truss, meshed_beam, True, frame_beam, frame_beam)
sys.Add(constr2)


force_fun = chrono.ChFunctionConst(0)
meshed_beam.AddForce(chrono.ChForceTorqueBody(force_fun, chrono.ChVector3d(0, 0, 0)))


truss2 = chrono.ChBody()
sys.AddBody(truss2)


constr3 = chrono.ChLinkMateGeneric()
constr3.Initialize(meshed_beam, truss2, True, frame_beam, frame_beam)
sys.Add(constr3)


force_fun2 = chrono.ChFunctionConst(0)
truss2.AddForce(chrono.ChForceTorqueBody(force_fun2, chrono.ChVector3d(0, 0, 0)))


if False:  
    vis_fem = chrono.ChVisualSystemIrrlicht()
    vis_fem.AttachSystem(fem_manager.GetSystem())
    vis_fem.SetWindowSize(1024, 768)
    vis_fem.SetWindowTitle('FEM manager demo')
    vis_fem.Initialize()
    vis_fem.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis_fem.AddSkyBox()
    vis_fem.AddCamera(chrono.ChVector3d(0, -1.5, 1), chrono.ChVector3d(0, 0, 0))
    vis_fem.AddTypicalLights()
    vis_fem.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 2.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam buckling')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -1.5, 1), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 2.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)


sys.SetSolverType(chrono.ChSolver.Type_SOR)
solver = sys.GetSolver()
solver.SetMaxIterations(50)
solver.SensitivityAnalyze()
solver.SetTolerance(1e-8)


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)