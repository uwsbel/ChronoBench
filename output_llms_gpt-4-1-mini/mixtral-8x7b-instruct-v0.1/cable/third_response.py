import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        builder = fea.ChBuilderCableANCF()
        body_offset = 0.05
        for i in range(n_chains):
            builder.BuildBeam(
                mesh,
                msection_cable2,
                10 + i,
                chrono.ChVector3d(0, 0, -0.1 - i * body_offset),
                chrono.ChVector3d(0.5, 0, -0.1 - i * body_offset)
            )

            # Apply forces and constraints
            last_node = builder.GetLastBeamNodes().back()
            last_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)

            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(last_node, mtruss)
            system.Add(constraint_hinge)

            # Connect to a box
            box = chrono.ChBody()
            box.SetBox(chrono.ChVector3d(0.05, 0.05, 0.05))
            box.SetPos(chrono.ChVector3d(0.5, 0, -0.1 - i * body_offset))
            system.Add(box)

            constraint_fixed = chrono.ChLinkLockRevolute()
            constraint_fixed.Initialize(last_node, box, chrono.ChVector3d(0, 0, 1))
            system.Add(constraint_fixed)

    def PrintBodyPositions(self, sys):
        for i in range(self.n_chains):
            last_node = self.beam_nodes[-(2 * (i + 1)) + 1]
            box = sys.GetBody(sys.GetNbBodies() - (i + 1))
            print(f"Chain {i + 1} end position: {box.GetPos()}")

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
    model.PrintBodyPositions(sys)