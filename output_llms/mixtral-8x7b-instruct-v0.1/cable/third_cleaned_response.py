import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


class Model1:
    def __init__(self, system, mesh):
        self.n_chains = 6  
        self.beam_length = 0.5
        self.beam_diameter = 0.015
        self.young_modulus = 0.01e9
        self.rayleigh_damping = 0.0001
        self.gravity = chrono.ChVector3d(0, -0.7, 0)
        self.step = 0.01

        self.beam_forces = [chrono.ChVector3d(0, -0.7, 0)] * self.n_chains
        self.box_size = 0.1

        self.bodies = []
        self.constraints = []

        self.mesh = mesh

        self.CreateChains(system)

    def CreateChains(self, system):
        offset = 0
        for i in range(self.n_chains):
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)

            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                self.mesh,
                self.msection_cable2,
                10,
                chrono.ChVector3d(offset, 0, -0.1),
                chrono.ChVector3d(offset + self.beam_length, 0, -0.1),
            )

            builder.GetLastBeamNodes().front().SetForce(self.beam_forces[i])

            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            self.constraints.append(constraint_hinge)

            box_body = chrono.ChBody()
            box_body.SetBox(chrono.ChVector3d(self.box_size / 2), chrono.ChVector3d(self.box_size, self.box_size, self.box_size))
            box_body.SetPos(chrono.ChVector3d(offset + self.beam_length, 0, -0.1))
            system.Add(box_body)

            constraint_weld = chrono.ChLinkWeld()
            constraint_weld.Initialize(builder.GetLastBeamNodes().back(), box_body)
            system.Add(constraint_weld)

            self.bodies.append(box_body)

            offset += self.beam_length + 0.05

    def PrintBodyPositions(self, system):
        for i, body in enumerate(self.bodies):
            pos = body.GetPos()
            print(f"Chain {i + 1} end body position: {pos}")



sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)  


visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
visualizebeamA.SetSmoothFaces(True)  
visualizebeamA.SetWireframe(False)  
mesh.AddVisualShapeFEA(visualizebeamA)  


visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
visualizebeamB.SetSymbolsThickness(0.006)  
visualizebeamB.SetSymbolsScale(0.01)  
visualizebeamB.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(visualizebeamB)  


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
    sys.DoStepDynamics(model.step)  
    model.PrintBodyPositions(sys)