import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.end_bodies = []

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        
        builder = fea.ChBuilderCableANCF()

        
        for chain_idx in range(n_chains):
            chain_length = 0.5 * (chain_idx + 1)
            start_point = chrono.ChVector3d(-0.5 * chain_idx, 0, -0.1)
            end_point = chrono.ChVector3d(-0.5 * chain_idx + chain_length, 0, -0.1)

            
            builder.BuildBeam(
                mesh,
                msection_cable2,
                10 + chain_idx,
                start_point,
                end_point
            )

            
            last_node = builder.GetLastBeamNodes().back()
            last_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetName(f"Truss reference {chain_idx}")
            system.AddBody(mtruss)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(last_node, mtruss)
            system.Add(constraint_hinge)

            
            box_body = chrono.ChBodyEasyBox(0.04, 0.04, 0.04, 1000, True, True)
            box_body.SetName(f"End box body {chain_idx}")
            box_body.SetPos(last_node.GetPos())
            box_body.SetFixed(True)
            system.AddBody(box_body)
            self.end_bodies.append(box_body)

            
            constraint_box = fea.ChLinkNodeFrame()
            constraint_box.Initialize(last_node, box_body)
            system.Add(constraint_box)

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f"Chain {i} end body position: {body.GetPos()}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
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


for body in model.end_bodies:
    box_shape = chrono.ChVisualShapeBox(0.045, 0.045, 0.045)
    box_shape.SetColor(chrono.ChColor(0.8, 0.1, 0.1))
    body.AddVisualShape(box_shape)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables with multiple chains')
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
    model.PrintBodyPositions()