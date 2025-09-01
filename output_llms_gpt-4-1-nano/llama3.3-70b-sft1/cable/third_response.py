import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []
        self.msection_cable2 = fea.ChBeamSectionCable()
        self.msection_cable2.SetDiameter(0.015)
        self.msection_cable2.SetYoungModulus(0.01e9)
        self.msection_cable2.SetRayleighDamping(0.0001)
        self.builder = fea.ChBuilderCableANCF()
        for i in range(n_chains):
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.system.Add(mtruss)
            self.builder.BuildBeam(
                mesh,
                self.msection_cable2,
                10 + i,
                chrono.ChVector3d(0, 0, -0.1 + i * 0.1),
                chrono.ChVector3d(0.5, 0, -0.1 + i * 0.1)
            )
            end_node = self.builder.GetLastBeamNodes().back()
            end_node.SetForce(chrono.ChVector3d(0, -0.7, 0))
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(end_node, mtruss)
            self.system.Add(constraint_hinge)
            end_body = chrono.ChBody()
            end_body.SetPos(end_node.GetPos())
            self.system.Add(end_body)
            self.end_bodies.append(end_body)
            constraint = chrono.ChLinkLock()
            constraint.Initialize(end_node, end_body, chrono.ChFramed(end_node.GetPos()))
            self.system.Add(constraint)
            vis_shape = chrono.ChVisualShapeBox()
            vis_shape.SetSize(chrono.ChVector3d(0.1, 0.1, 0.1))
            end_body.AddVisualShape(vis_shape)

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f'Chain {i+1} end body position: {body.GetPos()}')

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh)
sys.Add(mesh)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()