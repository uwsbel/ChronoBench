import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


print("Copyright (c) 2017 projectchrono.org")


sys = chrono.ChSystemSMC()


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)


chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)


vis = chronoirr.ChVisualSystemIrrlicht()


vis.AttachSystem(sys)


vis.SetWindowSize(1024, 768)


vis.SetWindowTitle("Beam Buckling Simulation")


vis.Initialize()


vis.AddCamera(chrono.ChVector3d(0, 3, 6))


vis.AddTypicalLights()


mesh = fea.ChMesh()


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(0.1, 0.1)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(1, 0, 0)))


mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)


mesh.AddElement(belement1)


sys.Add(mesh)


mfun = chrono.ChFunction_Sine(0.001, 1.5)


link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(hnode1, hnode2, chrono.ChFrameD())
sys.Add(link_motor)


link_motor.SetSpeedFunction(mfun)


link_constraint = chrono.ChLinkLockLock()
link_constraint.Initialize(hnode1, hnode2, chrono.ChFrameD())
sys.Add(link_constraint)


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


while vis.Run():
    
    sys.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()